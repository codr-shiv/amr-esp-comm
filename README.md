# Autonomous Mobile Robot (AMR) - ESP32 & Raspberry Pi micro-ROS Integration

**RASP PI IP (Anuj):** 10.78.121.82

This document chronicles the step-by-step process of establishing a flawless two-way ROS 2 communication bridge between an ESP32 microcontroller and a Raspberry Pi using micro-ROS over Wi-Fi (UDP).

---

## 1. Architecture Overview
*   **Raspberry Pi (The Agent):** Runs standard ROS 2 Humble on Ubuntu 22.04. It acts as the router/bridge, translating lightweight micro-ROS messages into standard ROS 2 topics.
*   **ESP32 (The Client):** Runs lightweight micro-ROS nodes via the Arduino IDE. It publishes sensor data and subscribes to motor commands.
*   **Transport Layer:** Wi-Fi (UDP) on port `8888`.

---

## 2. Raspberry Pi Setup (The micro-ROS Agent)

### A. ROS 2 Installation
We installed `ros-humble-ros-base` to save resources since the Pi is running headless.
During installation, we encountered an APT GPG key conflict (`Conflicting values set for option Signed-By`). 
**The Fix:** We wiped the old ROS repository configurations and regenerated a clean one:
```bash
sudo rm -f /etc/apt/sources.list.d/ros*.list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

### B. Building the Agent
We created a script to clone the `micro_ros_setup` package and build the agent. 
During the build process, a `ros2: command not found` error caused a partial/dirty build. This manifested later as a massive C++ linker error (`undefined reference to dds::xrce::...`).
**The Fix:** We cleared the corrupted build artifacts and re-compiled cleanly:
```bash
cd ~/microros_ws
rm -rf build/ install/ log/
colcon build
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

---

## 3. ESP32 Setup (The micro-ROS Client)

### A. Arduino IDE Preparation
1. Added the ESP32 Board Manager URL and installed Espressif's ESP32 boards.
2. Downloaded the `micro_ros_arduino` Humble release (v2.0.7-humble.zip) and installed it via **Sketch > Include Library > Add .ZIP Library**.

### B. Two-Way Communication Code
We wrote the `esp32_two_way.ino` script to prove bi-directional data flow.
**Key Architectural Concepts Implemented:**
*   **Publisher:** Registered to `esp_topic`. Sends a counter integer every 1000ms via a hardware timer.
*   **Subscriber:** Registered to `rpi_topic`. Listens for integers.
*   **Executor Sizing (CRITICAL):** Because the ESP32 is running both a publisher (timer) AND a subscriber, the Executor was explicitly initialized with **2 handles** (`RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));`).
*   **Callbacks:** Traffic routing is completely managed by the executor spinning in the `loop()`.

---

## 4. How to Run the Full Pipeline

### Step 1: Start the Agent on the Raspberry Pi
The agent must *always* be running for the ESP32 to communicate. Open an SSH terminal on the Pi:
```bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### Step 2: Boot the ESP32
Flash the ESP32 with `esp32_two_way.ino`. 
Open the **Arduino Serial Monitor** (115200 baud). Once connected to Wi-Fi and the Pi's Agent, it will print:
`micro-ROS Node Initialized and Ready!`

### Step 3: Verify Pi to ESP32 (Publishing to ESP)
Open a **new** SSH terminal on the Pi and send a command:
```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /rpi_topic std_msgs/msg/Int32 "{data: 1}"
```
*Verification:* The Arduino Serial Monitor will immediately display `Received command from Raspberry Pi: 1`.

### Step 4: Verify ESP32 to Pi (Subscribing to ESP)
In the Pi terminal, listen to the ESP32's data stream:
```bash
ros2 topic echo /esp_topic
```
*Verification:* You will see a live stream of ascending integers arriving from the ESP32.

---
**Status:** ✅ Fully Operational. Two-way bridge established successfully.