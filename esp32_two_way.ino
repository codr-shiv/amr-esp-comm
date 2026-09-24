#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

// ROS variables
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 pub_msg;
std_msgs__msg__Int32 sub_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// --- Publisher Callback ---
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
    pub_msg.data++; 
  }
}

// --- Subscriber Callback ---
void subscription_callback(const void * msgin)
{
  // Cast received message to Int32
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  
  // Print to Serial Monitor so you can see it from the ESP side!
  Serial.print("Received command from Raspberry Pi: ");
  Serial.println(msg->data);
  
  // Turn LED ON if we receive 1, OFF if 0
  if(msg->data == 1) {
    digitalWrite(LED_PIN, HIGH);
  } else if (msg->data == 0) {
    digitalWrite(LED_PIN, LOW);
  }
}

void setup() {
  // Start Serial to view logs in Arduino IDE
  Serial.begin(115200);
  
  // REMEMBER TO UPDATE YOUR WIFI CREDS HERE
  set_microros_wifi_transports("AMR", "AMR@ARTL", "192.168.0.103", 8888);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000); 

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

  // Initialize Publisher (Publishing to /esp32_status)
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "esp_topic"));

  // Initialize Subscriber (Listening to /esp32_cmd)
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "rpi_topic"));

  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // **IMPORTANT**: Executor now handles 2 things (1 timer AND 1 subscriber)
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscription_callback, ON_NEW_DATA));

  pub_msg.data = 0;
  Serial.println("micro-ROS Node Initialized and Ready!");
}

void loop() {
  delay(10);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
