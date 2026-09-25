// =====================================================================
//  AMR ESP32 FIRMWARE
//  Per-wheel velocity PID + quadrature encoders + micro-ROS over Wi-Fi
// ---------------------------------------------------------------------
//  ROS 2 interface (unchanged from the micro-ROS test code):
//    SUB  left_vel            std_msgs/Float64        left  wheel target (rad/s)
//    SUB  right_vel           std_msgs/Float64        right wheel target (rad/s)
//    PUB  encoder_telemetry   sensor_msgs/JointState  (best effort, 20 Hz)
//           name     = ["left_wheel", "right_wheel"]
//           position = [left angle (rad), right angle (rad)]    cumulative
//           velocity = [left (rad/s),    right (rad/s)]         filtered
//           header.stamp = Pi time (synchronised with the agent)
//
//  Architecture:
//    * controlTask (FreeRTOS, core 1, 50 Hz fixed): encoders -> PID -> PWM.
//      Never blocked by Wi-Fi / micro-ROS.
//    * loop() : micro-ROS (connect / reconnect / subscribe / publish)
//               + Serial debug & tuning commands.
//    * The two sides exchange data only through the `sh` struct, protected
//      by a spinlock.
//
//  Safety:
//    * Motors stop if the agent is not connected.
//    * Motors stop if left_vel / right_vel are not received for
//      CMD_TIMEOUT_MS (the Pi must publish them CONTINUOUSLY, >= 5 Hz).
//    * Automatic reconnection to the agent (no hang if the Pi is off).
//
//  Serial commands (115200 baud, line ending "Newline") - for bench tests:
//    ros           hand control back to ROS (default at boot)
//    s             STOP and hold (ROS commands ignored until "ros")
//    t 10 [5]      manual targets rad/s (left [right])  - ignores ROS
//    sq 10 2000    manual square wave 10 <-> 0 rad/s, period 2000 ms
//    pwm 300 300   manual OPEN-LOOP PWM (-1023..1023), PID bypassed
//    kp L 12       set parameter for L, R or B(oth): kp ki kff min max
//    p             print parameters
//    plot 0|1|2    0 = 1 Hz status text, 1 = speeds, 2 = speeds + PWM %
//
//  Libraries: micro_ros_arduino (matching your ROS 2 distro), ESP32Encoder
// =====================================================================

#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float64.h>
#include <sensor_msgs/msg/joint_state.h>

#include <ESP32Encoder.h>
#include "esp_arduino_version.h"

// ===================== Wi-Fi & AGENT CONFIGURATION =====================
#define SSID_NAME       "AMR"
#define SSID_PASSWORD   "AMR@ARTL"
#define AGENT_IP        "192.168.0.100"     // Raspberry Pi static IP
#define AGENT_PORT      8888

// ============================ ROS NAMES ================================
const char *NODE_NAME       = "esp32_wifi_wheel_node";
const char *TOPIC_LEFT_VEL  = "left_vel";
const char *TOPIC_RIGHT_VEL = "right_vel";
const char *TOPIC_TELEMETRY = "encoder_telemetry";

const uint32_t PUB_INTERVAL_MS = 50;    // telemetry at 20 Hz
const uint32_t CMD_TIMEOUT_MS  = 500;   // stop if no velocity command for this long (0 = never)
const bool     DEBUG_ROS_RX    = false; // true = print every received velocity (slows loop)

// ======================= LEFT = MOTOR 1 (26.9:1) =======================
const int   L_ENC_A = 32, L_ENC_B = 33, L_PWM = 18, L_DIR = 19, L_CH = 0;
const float L_CPR          = 752.6;
const bool  L_ENC_INVERT   = true;
const bool  L_MOTOR_INVERT = true;
const float L_KFF       = 49.72;
const float L_PWM_MIN   = 11.8;
const float L_KP        = 30.0;
const float L_KI        = 12.50;
const float L_MAX_SPEED = 17.00;

// ======================= RIGHT = MOTOR 2 (19.1:1) ======================
const int   R_ENC_A = 25, R_ENC_B = 26, R_PWM = 23, R_DIR = 22, R_CH = 1;
const float R_CPR          = 536.1;
const bool  R_ENC_INVERT   = false;
const bool  R_MOTOR_INVERT = false;
const float R_KFF       = 43.54;
const float R_PWM_MIN   = 12.0;
const float R_KP        = 25.0;
const float R_KI        = 10.0;
const float R_MAX_SPEED = 17.00;

// ============================ CONTROL ==================================
const int   PWM_FREQ    = 20000;
const int   PWM_BITS    = 10;
const int   PWM_MAX     = 1023;
const int   LOOP_HZ     = 50;      // control rate (20 ms)
const float VEL_ALPHA   = 0.3;     // speed low-pass filter
const float ACCEL_LIMIT = 40.0;    // rad/s^2 setpoint ramp
const float I_LIMIT     = 400.0;   // max PWM from the integral term

// =====================================================================
//                SHARED STATE  (loop  <->  controlTask)
// =====================================================================
enum CmdMode : uint8_t { MODE_ROS, MODE_SERIAL_VEL, MODE_SERIAL_PWM, MODE_SQUARE };

struct Params { float kff, pwmMin, kp, ki, maxSpeed; bool resetI; };

struct Shared {
  // ---- written by loop(), read by controlTask ----
  CmdMode  mode;
  bool     agentConnected;
  float    rosL, rosR;           // latest targets from ROS (rad/s)
  uint32_t rosLMs, rosRMs;       // when they arrived (millis)
  float    serL, serR;           // serial targets (rad/s) or raw PWM in MODE_SERIAL_PWM
  float    sqAmp;
  uint32_t sqPeriodMs, sqStartMs;
  Params   pL, pR;
  // ---- written by controlTask, read by loop() ----
  double   posL, posR;           // rad
  float    velL, velR;           // rad/s (filtered)
  float    tgtL, tgtR;           // target after limits
  float    refL, refR;           // ramped setpoint
  int      pwmL, pwmR;           // signed PWM output
};

Shared       sh;
portMUX_TYPE shMux = portMUX_INITIALIZER_UNLOCKED;
#define LOCK()   portENTER_CRITICAL(&shMux)
#define UNLOCK() portEXIT_CRITICAL(&shMux)

// =====================================================================
//                       MOTOR / ENCODER LAYER
//              (used ONLY by controlTask after setup)
// =====================================================================
struct Wheel {
  const char   *name;
  int           pwmPin, dirPin, ch;
  float         cpr;
  bool          encInvert, motorInvert;
  Params        prm;
  ESP32Encoder *enc;
  int64_t       lastCount;
  float         target, ref, meas, iTerm;
  double        pos;
  int           pwmOut;
};

ESP32Encoder encL, encR;
Wheel L, R;

void pwmSetup(int pin, int ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(ch, PWM_FREQ, PWM_BITS);
  ledcAttachPin(pin, ch);
#endif
}

void pwmWrite(int pin, int ch, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}

void initWheel(Wheel &w, const char *name, int encA, int encB, int pwmPin, int dirPin, int ch,
               float cpr, bool encInv, bool motInv, ESP32Encoder *enc) {
  w.name = name; w.pwmPin = pwmPin; w.dirPin = dirPin; w.ch = ch;
  w.cpr = cpr; w.encInvert = encInv; w.motorInvert = motInv; w.enc = enc;
  pinMode(dirPin, OUTPUT);
  pwmSetup(pwmPin, ch);
  enc->attachFullQuad(encA, encB);
  enc->setFilter(1023);
  enc->clearCount();
}

void setMotor(Wheel &w, int u) {
  u = constrain(u, -PWM_MAX, PWM_MAX);
  w.pwmOut = u;
  if (w.motorInvert) u = -u;
  digitalWrite(w.dirPin, u >= 0 ? HIGH : LOW);
  pwmWrite(w.pwmPin, w.ch, abs(u));
}

// true for a normal number (rejects NaN / inf / absurd values from the network)
static inline bool isValid(double x) { return (x == x) && x < 1e6 && x > -1e6; }

// scale both targets together if either exceeds its limit (keeps curvature)
void setTargets(float l, float r) {
  if (!isValid(l)) l = 0;
  if (!isValid(r)) r = 0;
  float s = 1.0f;
  if (fabsf(l) > L.prm.maxSpeed) s = fminf(s, L.prm.maxSpeed / fabsf(l));
  if (fabsf(r) > R.prm.maxSpeed) s = fminf(s, R.prm.maxSpeed / fabsf(r));
  L.target = l * s;
  R.target = r * s;
}

void updateWheel(Wheel &w, float dt, bool openLoop, int openLoopPwm) {
  // 1. measure
  int64_t c = w.enc->getCount();
  if (w.encInvert) c = -c;
  float raw = TWO_PI * (float)(c - w.lastCount) / (w.cpr * dt);
  w.lastCount = c;
  w.meas = VEL_ALPHA * raw + (1.0f - VEL_ALPHA) * w.meas;
  w.pos  = TWO_PI * (double)c / (double)w.cpr;

  if (openLoop) {
    w.target = 0; w.ref = 0; w.iTerm = 0;
    setMotor(w, openLoopPwm);
    return;
  }

  // 2. ramp
  float maxStep = ACCEL_LIMIT * dt;
  w.ref += constrain(w.target - w.ref, -maxStep, maxStep);

  if (w.target == 0.0f && fabsf(w.ref) < 1e-3f) {
    w.ref = 0; w.iTerm = 0;
    setMotor(w, 0);
    return;
  }

  // 3. feedforward + PI
  const Params &p = w.prm;
  float e    = w.ref - w.meas;
  float ff   = p.kff * w.ref + (w.ref > 0 ? p.pwmMin : (w.ref < 0 ? -p.pwmMin : 0));
  float pT   = p.kp * e;
  float iNew = constrain(w.iTerm + p.ki * e * dt, -I_LIMIT, I_LIMIT);
  float u    = ff + pT + iNew;

  // 4. anti-windup
  bool pushingHigher = (u >  PWM_MAX && e > 0);
  bool pushingLower  = (u < -PWM_MAX && e < 0);
  if (!pushingHigher && !pushingLower) w.iTerm = iNew;

  setMotor(w, (int)lroundf(ff + pT + w.iTerm));
}

// ---------------------- one control cycle ----------------------
void controlStep(float dt) {
  // 1. snapshot commands + parameters
  LOCK();
  Shared c = sh;
  sh.pL.resetI = false;
  sh.pR.resetI = false;
  UNLOCK();

  L.prm = c.pL; if (c.pL.resetI) L.iTerm = 0;
  R.prm = c.pR; if (c.pR.resetI) R.iTerm = 0;

  // 2. decide what the wheels should do
  uint32_t nowMs = millis();
  bool openLoop = false;
  switch (c.mode) {
    case MODE_ROS: {
      bool fresh = c.agentConnected &&
                   (CMD_TIMEOUT_MS == 0 ||
                    ((nowMs - c.rosLMs) <= CMD_TIMEOUT_MS && (nowMs - c.rosRMs) <= CMD_TIMEOUT_MS));
      if (fresh) setTargets(c.rosL, c.rosR);
      else       setTargets(0, 0);
      break;
    }
    case MODE_SERIAL_VEL:
      setTargets(c.serL, c.serR);
      break;
    case MODE_SQUARE: {
      uint32_t per = c.sqPeriodMs < 200 ? 2000 : c.sqPeriodMs;
      bool high = ((nowMs - c.sqStartMs) % per) < per / 2;
      float a = high ? c.sqAmp : 0.0f;
      setTargets(a, a);
      break;
    }
    case MODE_SERIAL_PWM:
      openLoop = true;
      break;
  }

  // 3. run both wheels
  updateWheel(L, dt, openLoop, (int)c.serL);
  updateWheel(R, dt, openLoop, (int)c.serR);

  // 4. hand the results to loop()
  LOCK();
  sh.posL = L.pos;    sh.posR = R.pos;
  sh.velL = L.meas;   sh.velR = R.meas;
  sh.tgtL = L.target; sh.tgtR = R.target;
  sh.refL = L.ref;    sh.refR = R.ref;
  sh.pwmL = L.pwmOut; sh.pwmR = R.pwmOut;
  UNLOCK();
}

// ---------------------- 50 Hz control task (core 1) ----------------------
void controlTask(void *) {
  const TickType_t period = pdMS_TO_TICKS(1000 / LOOP_HZ);
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t   lastUs   = micros();

  for (;;) {
    vTaskDelayUntil(&lastWake, period);   // exact 20 ms period, independent of Wi-Fi
    uint32_t nowUs = micros();
    float dt = (nowUs - lastUs) / 1e6f;
    lastUs = nowUs;
    if (dt > 0.0f) controlStep(dt);
  }
}

// =====================================================================
//                           micro-ROS LAYER
// =====================================================================
rclc_support_t     support;
rcl_allocator_t    allocator;
rcl_node_t         node;
rclc_executor_t    executor;
rcl_subscription_t sub_left_vel;
rcl_subscription_t sub_right_vel;
rcl_publisher_t    pub_encoder;
bool               supportReady = false;

std_msgs__msg__Float64        msg_left_vel;
std_msgs__msg__Float64        msg_right_vel;
sensor_msgs__msg__JointState  msg_encoder;

// static storage for the JointState message (no malloc)
char                      nameLeft[]  = "left_wheel";
char                      nameRight[] = "right_wheel";
char                      frameId[]   = "";
rosidl_runtime_c__String  jointNames[2];
double                    jointPos[2];
double                    jointVel[2];

enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState agentState = WAITING_AGENT;

void initTelemetryMsg() {
  jointNames[0].data = nameLeft;  jointNames[0].size = strlen(nameLeft);  jointNames[0].capacity = sizeof(nameLeft);
  jointNames[1].data = nameRight; jointNames[1].size = strlen(nameRight); jointNames[1].capacity = sizeof(nameRight);

  msg_encoder.header.frame_id.data     = frameId;
  msg_encoder.header.frame_id.size     = 0;
  msg_encoder.header.frame_id.capacity = sizeof(frameId);

  msg_encoder.name.data     = jointNames; msg_encoder.name.size     = 2; msg_encoder.name.capacity     = 2;
  msg_encoder.position.data = jointPos;   msg_encoder.position.size = 2; msg_encoder.position.capacity = 2;
  msg_encoder.velocity.data = jointVel;   msg_encoder.velocity.size = 2; msg_encoder.velocity.capacity = 2;
  msg_encoder.effort.data   = NULL;       msg_encoder.effort.size   = 0; msg_encoder.effort.capacity   = 0;
}

void left_vel_callback(const void *msgin) {
  const std_msgs__msg__Float64 *m = (const std_msgs__msg__Float64 *)msgin;
  if (!isValid(m->data)) return;
  uint32_t now = millis();
  LOCK(); sh.rosL = (float)m->data; sh.rosLMs = now; UNLOCK();
  if (DEBUG_ROS_RX) Serial.printf("-> left_vel  %.3f\n", m->data);
}

void right_vel_callback(const void *msgin) {
  const std_msgs__msg__Float64 *m = (const std_msgs__msg__Float64 *)msgin;
  if (!isValid(m->data)) return;
  uint32_t now = millis();
  LOCK(); sh.rosR = (float)m->data; sh.rosRMs = now; UNLOCK();
  if (DEBUG_ROS_RX) Serial.printf("-> right_vel %.3f\n", m->data);
}

bool createEntities() {
  allocator     = rcl_get_default_allocator();
  node          = rcl_get_zero_initialized_node();
  sub_left_vel  = rcl_get_zero_initialized_subscription();
  sub_right_vel = rcl_get_zero_initialized_subscription();
  pub_encoder   = rcl_get_zero_initialized_publisher();
  executor      = rclc_executor_get_zero_initialized_executor();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  supportReady = true;
  if (rclc_node_init_default(&node, NODE_NAME, "", &support) != RCL_RET_OK) return false;

  if (rclc_subscription_init_default(&sub_left_vel, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), TOPIC_LEFT_VEL) != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(&sub_right_vel, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), TOPIC_RIGHT_VEL) != RCL_RET_OK) return false;
  if (rclc_publisher_init_best_effort(&pub_encoder, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), TOPIC_TELEMETRY) != RCL_RET_OK) return false;

  if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &sub_left_vel, &msg_left_vel,
        &left_vel_callback, ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &sub_right_vel, &msg_right_vel,
        &right_vel_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

  rmw_uros_sync_session(1000);   // align ESP32 clock with the Pi for timestamps
  return true;
}

void destroyEntities() {
  if (supportReady) {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  }
  (void)rcl_publisher_fini(&pub_encoder, &node);
  (void)rcl_subscription_fini(&sub_left_vel, &node);
  (void)rcl_subscription_fini(&sub_right_vel, &node);
  (void)rclc_executor_fini(&executor);
  (void)rcl_node_fini(&node);
  if (supportReady) (void)rclc_support_fini(&support);
  supportReady = false;
}

void setAgentConnected(bool connected) {
  LOCK();
  sh.agentConnected = connected;
  if (!connected) { sh.rosL = 0; sh.rosR = 0; }   // never resume old targets after reconnect
  UNLOCK();
}

void publishTelemetry() {
  LOCK();
  double pl = sh.posL, pr = sh.posR;
  float  vl = sh.velL, vr = sh.velR;
  UNLOCK();

  jointPos[0] = pl;  jointPos[1] = pr;
  jointVel[0] = vl;  jointVel[1] = vr;

  if (rmw_uros_epoch_synchronized()) {
    int64_t ns = rmw_uros_epoch_nanos();
    msg_encoder.header.stamp.sec     = (int32_t)(ns / 1000000000LL);
    msg_encoder.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
  } else {
    uint32_t ms = millis();
    msg_encoder.header.stamp.sec     = ms / 1000;
    msg_encoder.header.stamp.nanosec = (ms % 1000) * 1000000UL;
  }
  (void)rcl_publish(&pub_encoder, &msg_encoder, NULL);
}

void microRosStep() {
  static uint32_t lastPingMs = 0, lastPubMs = 0;
  uint32_t now = millis();

  switch (agentState) {
    case WAITING_AGENT:
      if (now - lastPingMs >= 500) {
        lastPingMs = now;
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) agentState = AGENT_AVAILABLE;
      }
      break;

    case AGENT_AVAILABLE:
      if (createEntities()) {
        agentState = AGENT_CONNECTED;
        setAgentConnected(true);
        Serial.println("# micro-ROS: CONNECTED to agent");
      } else {
        destroyEntities();
        agentState = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED:
      if (now - lastPingMs >= 1000) {
        lastPingMs = now;
        if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) { agentState = AGENT_DISCONNECTED; break; }
      }
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
      if (now - lastPubMs >= PUB_INTERVAL_MS) {
        lastPubMs = now;
        publishTelemetry();
      }
      break;

    case AGENT_DISCONNECTED:
      setAgentConnected(false);
      destroyEntities();
      Serial.println("# micro-ROS: agent LOST - motors stopped, retrying...");
      agentState = WAITING_AGENT;
      break;
  }
}

// =====================================================================
//                     SERIAL DEBUG / TUNING LAYER
// =====================================================================
int plotMode = 0;

void printParams() {
  LOCK(); Params a = sh.pL, b = sh.pR; UNLOCK();
  Serial.printf("# L: kff=%.2f min=%.1f kp=%.2f ki=%.2f max=%.1f rad/s  cpr=%.1f\n",
                a.kff, a.pwmMin, a.kp, a.ki, a.maxSpeed, L_CPR);
  Serial.printf("# R: kff=%.2f min=%.1f kp=%.2f ki=%.2f max=%.1f rad/s  cpr=%.1f\n",
                b.kff, b.pwmMin, b.kp, b.ki, b.maxSpeed, R_CPR);
}

bool setParam(Params &p, const char *key, float v) {
  if      (!strcmp(key, "kp"))  p.kp = v;
  else if (!strcmp(key, "ki"))  p.ki = v;
  else if (!strcmp(key, "kff")) p.kff = v;
  else if (!strcmp(key, "min")) p.pwmMin = v;
  else if (!strcmp(key, "max")) p.maxSpeed = v;
  else return false;
  p.resetI = true;
  return true;
}

void setMode(CmdMode m, float l, float r) {
  LOCK(); sh.mode = m; sh.serL = l; sh.serR = r; UNLOCK();
}

void runCommand(char *s) {
  float a, b; char key[8], who;

  if (!strcmp(s, "ros"))                          { setMode(MODE_ROS, 0, 0); Serial.println("# control -> ROS"); }
  else if (!strcmp(s, "s"))                       { setMode(MODE_SERIAL_VEL, 0, 0); Serial.println("# STOPPED (ROS ignored). Type 'ros' to resume ROS control"); }
  else if (!strcmp(s, "p"))                       { printParams(); }
  else if (sscanf(s, "t %f %f", &a, &b) == 2)     { setMode(MODE_SERIAL_VEL, a, b); }
  else if (sscanf(s, "t %f", &a) == 1)            { setMode(MODE_SERIAL_VEL, a, a); }
  else if (sscanf(s, "sq %f %f", &a, &b) == 2 || sscanf(s, "sq %f", &a) == 1) {
    if (sscanf(s, "sq %f %f", &a, &b) != 2) b = 2000;
    uint32_t now = millis();
    LOCK();
    sh.mode = (a == 0) ? MODE_SERIAL_VEL : MODE_SQUARE;
    sh.serL = sh.serR = 0;
    sh.sqAmp = a; sh.sqPeriodMs = (b >= 200) ? (uint32_t)b : 2000; sh.sqStartMs = now;
    UNLOCK();
  }
  else if (sscanf(s, "pwm %f %f", &a, &b) == 2)   { setMode(MODE_SERIAL_PWM, constrain(a, -PWM_MAX, PWM_MAX), constrain(b, -PWM_MAX, PWM_MAX)); }
  else if (sscanf(s, "plot %f", &a) == 1)         { plotMode = (int)a; }
  else if (sscanf(s, "%7s %c %f", key, &who, &a) == 3) {
    who = toupper(who);
    bool ok = true;
    LOCK();
    if (who == 'L' || who == 'B') ok &= setParam(sh.pL, key, a);
    if (who == 'R' || who == 'B') ok &= setParam(sh.pR, key, a);
    UNLOCK();
    if (!ok) Serial.printf("# unknown parameter '%s'\n", key);
    printParams();
  }
  else Serial.printf("# ? '%s'\n", s);
}

void handleSerial() {
  static char buf[48];
  static int  n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (n) { buf[n] = 0; runCommand(buf); n = 0; }
    } else if (n < (int)sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

void serialOutput() {
  static uint32_t lastPlot = 0, lastStatus = 0;
  uint32_t now = millis();

  LOCK(); Shared c = sh; UNLOCK();

  if (plotMode == 1 && now - lastPlot >= 40) {
    lastPlot = now;
    Serial.printf("L_ref:%.2f,L_meas:%.2f,R_ref:%.2f,R_meas:%.2f\n", c.refL, c.velL, c.refR, c.velR);
  } else if (plotMode == 2 && now - lastPlot >= 40) {
    lastPlot = now;
    Serial.printf("L_ref:%.2f,L_meas:%.2f,R_ref:%.2f,R_meas:%.2f,L_pwm%%:%.1f,R_pwm%%:%.1f\n",
                  c.refL, c.velL, c.refR, c.velR, 100.0f * c.pwmL / PWM_MAX, 100.0f * c.pwmR / PWM_MAX);
  } else if (plotMode == 0 && now - lastStatus >= 1000) {
    lastStatus = now;
    static const char *modeName[] = { "ROS", "SERIAL", "PWM", "SQUARE" };
    Serial.printf("[%s|%s] tgt L=%6.2f R=%6.2f | meas L=%6.2f R=%6.2f rad/s | pos L=%8.2f R=%8.2f rad | pwm L=%5d R=%5d\n",
                  c.agentConnected ? "ROS OK " : "NO AGENT", modeName[c.mode],
                  c.tgtL, c.tgtR, c.velL, c.velR, c.posL, c.posR, c.pwmL, c.pwmR);
  }
}

// =====================================================================
//                             SETUP / LOOP
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n# === AMR ESP32 firmware: PID + encoders + micro-ROS ===");

  // ---- motors & encoders (motors held at 0) ----
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  initWheel(L, "L", L_ENC_A, L_ENC_B, L_PWM, L_DIR, L_CH, L_CPR, L_ENC_INVERT, L_MOTOR_INVERT, &encL);
  initWheel(R, "R", R_ENC_A, R_ENC_B, R_PWM, R_DIR, R_CH, R_CPR, R_ENC_INVERT, R_MOTOR_INVERT, &encR);
  setMotor(L, 0);
  setMotor(R, 0);

  // ---- shared state ----
  sh.mode = MODE_ROS;
  sh.agentConnected = false;
  sh.pL = { L_KFF, L_PWM_MIN, L_KP, L_KI, L_MAX_SPEED, true };
  sh.pR = { R_KFF, R_PWM_MIN, R_KP, R_KI, R_MAX_SPEED, true };
  L.prm = sh.pL;
  R.prm = sh.pR;
  printParams();

  // ---- start the 50 Hz control loop (keeps motors safe while Wi-Fi connects) ----
  xTaskCreatePinnedToCore(controlTask, "control", 4096, NULL, 3, NULL, 1);

  // ---- Wi-Fi + micro-ROS transport ----
  Serial.print("# Wi-Fi connecting");
  WiFi.begin(SSID_NAME, SSID_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(100); Serial.print("."); }
  Serial.printf("\n# Wi-Fi connected, ESP32 IP = %s\n", WiFi.localIP().toString().c_str());

  set_microros_wifi_transports((char *)SSID_NAME, (char *)SSID_PASSWORD, (char *)AGENT_IP, AGENT_PORT);
  WiFi.setSleep(false);   // disable modem sleep: much lower UDP latency
  delay(500);

  initTelemetryMsg();
  agentState = WAITING_AGENT;
  Serial.printf("# waiting for micro-ROS agent at %s:%d ...\n", AGENT_IP, AGENT_PORT);
}

void loop() {
  microRosStep();
  handleSerial();
  serialOutput();
  delay(1);   // yield: lets lower-priority tasks run, ~1 ms extra latency at most
}