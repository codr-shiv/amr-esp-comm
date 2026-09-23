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
*These commands must be run on the Raspberry Pi via SSH.*

### Step 1: Install Prerequisites
```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool git build-essential
sudo rosdep init
rosdep update
```

### Step 2: Create the Workspace
```bash
mkdir -p ~/microros_ws/src
cd ~/microros_ws
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
```

### Step 3: Install Dependencies
```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
```

### Step 4: Build the Setup Tool & The Agent
```bash
colcon build
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

### Step 5: Make it Permanent
```bash
echo "source ~/microros_ws/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 3. ESP32 Setup (The micro-ROS Client)

### Step 1: Arduino IDE Preparation
1. Go to **File > Preferences** and add the ESP32 Board Manager URL: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
2. Go to **Tools > Board > Boards Manager**, search for `esp32` (by Espressif) and install it.
3. Download the `micro_ros_arduino` Humble release (`v2.0.7-humble.zip`) from their GitHub releases page.
4. Install it via **Sketch > Include Library > Add .ZIP Library**.

### Step 2: The Two-Way Communication Code
We wrote the `esp32_two_way.ino` script to prove bi-directional data flow.
**Key Architectural Concepts Implemented:**
*   **Publisher:** Registered to `esp_topic`. Sends a counter integer every 1000ms.
*   **Subscriber:** Registered to `rpi_topic`. Listens for integers and toggles an LED.
*   **Executor Sizing (CRITICAL):** Because the ESP32 is running both a publisher (timer) AND a subscriber, the Executor was explicitly initialized with **2 handles**:
    ```cpp
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    ```

### Step 3: Flash the Code
1. Open `esp32_two_way.ino`.
2. Update the Wi-Fi SSID and Password to match the network the Raspberry Pi is on.
3. Connect the ESP32 to your computer via USB and hit **Upload**.

---

## 4. How to Run the Full Pipeline

### Step 1: Start the Agent on the Raspberry Pi
The agent must *always* be running for the ESP32 to communicate. Open an SSH terminal on the Pi:
```bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### Step 2: Boot the ESP32
Ensure the ESP32 is powered on. Open the **Arduino Serial Monitor** (115200 baud). Once connected to Wi-Fi and the Pi's Agent, it will print:
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

## 5. Troubleshooting & Errors Encountered

During the setup, we encountered several common pitfalls. Here is how we resolved them:

### Error 1: APT GPG Key Conflict
**The Problem:** When running `sudo apt update` to install ROS 2, the system threw:
`E: Conflicting values set for option Signed-By regarding source http://packages.ros.org/ros2/ubuntu/ jammy...`
**The Cause:** An older ROS repository file (`.list`) existed in `/etc/apt/sources.list.d/` with an inline PGP key, conflicting with the modern keyring method.
**The Fix:** We wiped all ROS apt lists and cleanly regenerated the modern `ros2.list`:
```bash
sudo rm -f /etc/apt/sources.list.d/ros*.list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

### Error 2: The "Dirty Build" Linker Error
**The Problem:** When running `ros2 run micro_ros_setup build_agent.sh`, the compiler crashed with massive C++ linker errors:
`/usr/bin/ld: ... undefined reference to dds::xrce::...`
**The Cause:** A previous attempt to build the agent was executed *without* first sourcing the ROS 2 base installation (`source /opt/ros/humble/setup.bash`). This caused `colcon` to generate corrupted build artifacts that lingered in the workspace.
**The Fix:** We wiped the corrupted build folders and re-compiled cleanly:
```bash
cd ~/microros_ws
rm -rf build/ install/ log/
colcon build
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

### Error 3: No Subscribers Found
**The Problem:** When testing the Pi-to-ESP communication using `ros2 topic pub --once /rpi_topic std_msgs/msg/Int32 "{data: 1}"`, the terminal got stuck printing:
`Waiting for at least 1 matching subscription(s)...`
**The Cause:** The `micro_ros_agent` process had been killed (`^C`) in the terminal. Without the agent acting as a bridge, the ESP32 lost its connection to the ROS network and entered a crash loop.
**The Fix:** 
1. Restarted the agent (`ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`) and *left it running*.
2. Physically pressed the `RST` (reset) button on the ESP32 to reboot it so it could find the newly started agent.
3. Used a *second* terminal to run the `ros2 topic pub` command.