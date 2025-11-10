# JetBot-SLAM Setup Guide (Jetson Nano)

This repository contains the full setup, firmware, and ROS 2 infrastructure for
running **JetBot-SLAM** on a **Waveshare JetBot** powered by an **NVIDIA Jetson Nano (4 GB)**.

The system combines:
- Embedded firmware (Teensy 3.2)
- ROS 2 Foxy motor + wheel odometry nodes
- Intel RealSense integration
- Foxglove visualization
- Fully containerized deployment using Docker

This README is both:
- a **first-time bring-up guide**, and
- a **developer reference** for the motor + odometry stack.

---

## 0. Repository Structure
```
jetbot-slam/
├── containers/
│ ├── ros-foxy-jetbot-dev/
│ │ ├── Dockerfile
│ │ ├── docker-compose.jetbot.yml
│ │ └── entrypoint.sh
│ │
│ ├── ros-foxy-realsense/
│ │ ├── Dockerfile.foxy
│ │ ├── docker-compose.realsense.yml
│ │ ├── realsense_lowpower.yaml
│ │ └── ros_realsense_entrypoint.sh
│ │
│ └── ros-galactic-foxglove/
│ ├── Dockerfile.galactic
│ ├── docker-compose.foxglove.yml
│ └── ros_foxglove_entrypoint.sh
│
├── docker-compose.yml
├── Dockerfile.noetic
│
├── realsense_support/
│ └── 0001-Fix-patch-to-support-L4T-32.7.6.patch
│
├── ros2_ws/
│ └── src/
│ └── jetbot_base/
│ ├── CMakeLists.txt
│ ├── package.xml
│ ├── config/
│ │ └── wheel_odom.yaml
│ ├── include/
│ │ └── jetbot_base/
│ │ ├── jetbot_motor_driver.hpp
│ │ └── wheel_odom_node.hpp
│ ├── src/
│ │ ├── jetbot_motor_driver.cpp
│ │ ├── jetbot_motor_driver_node.cpp
│ │ ├── wheel_odom_node.cpp
│ │ └── wheel_odom_main.cpp
│ └── launch/
│ ├── jetbot_motor_driver.launch.py
│ └── wheel_odom.launch.py
│
├── teensy3.2-wheel-odom/
│ ├── include/
│ │ ├── WheelEncoders.h
│ │ ├── protocol.hpp
│ │ ├── crc16.hpp
│ │ ├── params.hpp
│ │ └── commands.hpp
│ ├── src/
│ │ ├── main.cpp
│ │ ├── WheelEncoders.cpp
│ │ ├── params.cpp
│ │ └── commands.cpp
│ └── platformio.ini
│
└── README.md
```

## 1. Flash the Jetson Nano Image

### 1.1 Download Base Image
Get NVIDIA’s JetPack 4.6.1 image (`jetson-nano-jp461-sd-card-image`) from
https://developer.nvidia.com/embedded/l4t/r32_release_v7.1/jp_4.6.1_b110_sd_card/jeston_nano/jetson-nano-jp461-sd-card-image.zip
* If link breaks, find latest `Jetson Nano Developer Kit SD Card Image` @
https://developer.nvidia.com/embedded/downloads.

### 1.2 Flash SD Card
Unzip the SD card image if necessary and insert a micro SD card (>64 GB) into a
host computer (assuming Linux or Mac). Identify the device name:
```bash
lsblk
```
Look for a device such as `/dev/sdX` or `/dev/mmcblk0` (do not use the partition number, e.g., `/dev/sdX1`). **Be sure you don't select your host's hard drive!**
Assuming the image is named `jetson-nano-jp461-sd-card-image.img` and the card
is `/dev/sdX`, run:
```bash
# Unmount all partitions before overwriting
sudo umount /dev/sdX*

# Write the image to the SD card
sudo dd if=jetson-nano-4gb-jp46-sd-card-image.img of=/dev/sdX bs=4M status=progress conv=fsync

# Sync after writing
sync

# Safely eject the card
sudo eject /dev/sdX
```

### 1.3 First Boot (Headless via Micro-USB)

If you are running headless (no monitor or keyboard), you can complete the
first-boot setup over the Jetson Nano’s **micro-USB port**, which enumerates as
a USB-serial device when powered.

#### 1.3.1  Connect and Identify the Device
1. Insert the flashed micro-SD card into the Nano.  
2. Connect the **micro-USB port** on the Nano to your host computer.  
3. Make sure the Jetbot AI Kit power board is plugged in and turn the switch ON. 
4. On the host PC, identify the serial device:
   ```bash
   ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
   ```
It will typically appear as `/dev/ttyACM0`.

#### 1.3.2 Open a Serial Console
On Linux or Mac:
```bash
screen /dev/ttyACM0 115200
```
or, if using minicom:
```bash
sudo minicom -D /dev/ttyACM0 -b 115200
```
If you see no output, press **Enter** once after powering the board.

#### 1.3.3 Complete NVIDIA Setup Wizard
The serial console exposes the same first-boot prompts that appear on HDMI.
Follow the prompts to create your user and connect the Jetson to Wi-Fi. Once
the initialization

---

## 2. Initial Configuration and SSH

### 2.1 Create User and Enable SSH
```bash
sudo usermod -aG sudo,adm,video,plugdev,netdev,gpio,i2c $USER
sudo systemctl enable ssh
sudo systemctl start ssh
```

### 2.2 Add SSH Key
```bash
mkdir -p ~/.ssh
chmod 700 ~/.ssh
nano ~/.ssh/authorized_keys
chmod 600 ~/.ssh/authorized_keys
```
*(Paste your public key into `authorized_keys`.)*

### 2.3 Test SSH
```bash
ssh <your-username>@<nano_ip>
```

---

## 3. [Optional] Restore JetBot Utilities
This allows you to have the latest NVIDIA-supported SD card image but also have
all the Jetbot AI Kit functionality.

### 3.1 Create a JetBot user
```bash
sudo adduser jetbot
sudo usermod -aG sudo,adm,video,plugdev,netdev,gpio,i2c jetbot

# Switch to the jetbot user
sudo su jetbot
```

### 3.2 [Optional] Add SSH Key for jetbot
```bash
mkdir -p ~/.ssh
chmod 700 ~/.ssh
nano ~/.ssh/authorized_keys
chmod 600 ~/.ssh/authorized_keys
```

### 3.3 Setup JetBot Environment
Clone NVIDIA's AI-IOT jetbot repo

```bash
git clone https://github.com/NVIDIA-AI-IOT/jetbot.git
cd jetbot
```

and follow the native install instructions found at
https://github.com/NVIDIA-AI-IOT/jetbot/blob/master/docs/software_setup/native_setup.md.

---

## 4. Power and Cooling Performance
```bash
sudo nvpmodel -m 0           # Max performance
sudo jetson_clocks           # Lock clock speeds
sudo systemctl enable --now nvfancontrol
```

---

## 5. Install Docker with NVIDIA Runtime

### 5.1 Install and Enable
```bash
sudo apt install -y docker.io
sudo systemctl enable --now docker
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

### 5.2 Verify
```bash
docker --version
sudo docker info | grep -i runtime
```

### 5.3 Configure NVIDIA Runtime (if missing)
```bash
sudo nano /etc/docker/daemon.json
```
Add:
```json
{
  "default-runtime": "nvidia",
  "runtimes": {
    "nvidia": {
      "path": "/usr/bin/nvidia-container-runtime",
      "runtimeArgs": []
    }
  }
}
```
Then restart Docker:
```bash
sudo systemctl restart docker
```

---

## 6. Clone Jetbot-SLAM repo to Jetson Nano
```bash
mkdir -p ~/workspace/
cd ~/workspace/
git clone https://github.com/zmswanson/jetbot-slam.git
cd jetbot-slam
```

---

## 7. Patch Jetson Kernels for librealsense
Clone the librealsense library to v2.54.1 (as used in Dockerfile.foxy)

```bash
git clone -b v2.54.1 https://github.com/IntelRealSense/librealsense.git ~/workspace/librealsense
```

Apply the patch 0001-Fix-patch-to-support-L4T-32.7.6.patch to librealsense.

```bash
patch -p1 ~/workspace/librealsense/scripts/patch-realsense-ubuntu-L4T.sh < 0001-Fix-patch-to-support-L4T-32.7.6.patch
```

Run the following scripts to set up the Jetson host for use with librealsense:

```bash
cd ~/workspace/librealsense/
./scripts/setup_udev_rules.sh && ./scripts/patch-realsense-ubuntu-L4T.sh
```

Follow any prompts.

---

## 8. Docker & ROS Infrastructure

All ROS components run inside Docker containers.

Containers are separated by function:

* `ros-foxy-jetbot-dev`: motor + wheel odometry
* `ros-foxy-realsense`: Intel RealSense driver
* `ros-galactic-foxglove`: visualization

Containers may be built as follows using jetbot-dev as an example:
``` bash
docker-compose -f ./containers/ros-foxy-jetbot-dev/docker-compose.jetbot.yml build # optional: add --no-cache flag to build from scratch
```

Containers may be launched in the background as:
``` bash
docker-compose -f ./containers/ros-foxy-jetbot-dev/docker-compose.jetbot.yml up -d
```

You may interactively engage with a Bash shell in the container as:
``` bash
docker exec -it foxy_jetbot_dev bash
```

The containers may be stopped as:
``` bash
docker-compose -f ./containers/ros-foxy-jetbot-dev/docker-compose.jetbot.yml down # optional: add the --remove-orphans flag for additional cleanup
```

**TODO**: Combine Realsense and Jetbot-Dev containers.

**TODO**: Make Realsense container easier to reconfigure camera settings.

---

## 9 Teensy Firmware Programming via Jetson
The platform.io project includes upload capabilities to scp the firmware file to
the Jetson Nano and then run an SSH command to launch the Teensy Loader and
write the firmware to the Teensy device without requiring any physical contact
with the robot.

Note that this will require (a) you having your SSh keys properly applied on
the Jetson Nano (see Section 2) and (b) changing `./teensy3.2-wheel-odom/platformio.ini` to use your username and networking.

### 9.1 Build and Install Teensy Loader CLI
```bash
cd ~/workspace
git clone -b 2.3 https://github.com/PaulStoffregen/teensy_loader_cli.git
cd teensy_loader_cli
make
sudo cp teensy_loader_cli /usr/local/bin/
```

### 9.2 Add Permissions
```bash
sudo usermod -aG dialout $USER
sudo usermod -aG plugdev $USER
```
Reboot or re-login.

### 9.3 Connect and Verify
```bash
lsusb | grep Teensy
dmesg | grep ttyACM
ls /dev/ttyACM*
screen /dev/ttyACM0 115200
```

### 9.4 Flash Firmware
```bash
sudo teensy_loader_cli --mcu=TEENSY32 -w -v /home/jetbot/firmware.hex
```

---

## 10. Wiring Diagram (Teensy ↔ Jetson Nano)

| Jetson Nano Pin | Function | Teensy 3.2 Pin | Notes |
|-----------------|-----------|----------------|-------|
| Pin 1 (3.3 V)   | Power     | VIN (3.3 V)    | Power Teensy from Jetson |
| Pin 6 (GND)     | Ground    | GND            | Common ground |
| Pin 3 (SDA)     | I²C Data  | 18 (SDA0)      | For JetBot monitor |
| Pin 5 (SCL)     | I²C Clock | 19 (SCL0)      | Use pull-ups if needed |
| Pin 8 (TXD)     | UART TX   | 0 (RX1)        | Optional serial |
| Pin 10 (RXD)    | UART RX   | 1 (TX1)        | Optional serial |

TODO: Upload visual diagram, including motor to Teensy wiring and power wiring.

TODO: Include odometry enabled motors that I added to the Jetbot AI Kit.

---

## 11. Teensy 3.2 Wheel Encoder Firmware

### 11.1 Hardware Overview
* Teensy 3.2
* Quadrature encoders on left/right wheels (`DFRobot FIT0450`)
* UART connection to Jetson via `/dev/ttyTHS1`
* Forward motion = positive ticks on both wheels

### 11.2 Firmware Features
The Teensy firmware:
* Reads quadrature encoders
* Computes delta ticks + dt
* Streams data over UART using a binary protocol. Includes:
    - sync word
    - sequence number
    - CRC16-CCITT
* Supports an ASCII command channel on the same UART
* Persists parameters in EEPROM

Binary mode is default; JSON mode is available for debugging.

### 11.3 Command Channel
Commands accepted over `/dev/THS1`:
```
HELP
PING
GET_PARAMS
SET_PARAM counts_per_rev <float>
SET_PARAM report_hz <int>
SET_PARAM invert_left <+1|-1>
SET_PARAM invert_right <+1|-1>
RATE <hz>
MODE BIN
MODE JSON
RESET_ODOM
SAVE_PARAMS
LOAD_PARAMS
```

---

## 12. ROS 2 Base Drivers (`jetbot_base`)
### 12.1 Motor Driver Node
* Publishes motor commands to the JetBot via /dev/i2c-1
* Motor control based off Adafruit MotorHat control used in original Jetbot code
* Runs inside the `ros-foxy-jetbot-dev` container
* Currently independent of odometry and localization

### 12.2 Wheel Odometry Node (`jetbot_wheel_odom`)
Consumes the Teensy UART stream and publishes raw wheel odometry.

**Key Features**
* Robust binary parsing with resynchronization
* Pose integration from delta ticks (midpoint method)
* Publishes:
    - `/wheel/odom/` (`nav_msgs/Odometry`)
    - `/diagnostics` (`diagnostic_msgs/DiagnosticArray`)
    - Optional TF broadcast (`odom -> base_link`, disabled by default)

When using `robot_localization`, this node should not publish TF, as the EKF should own `odom -> base_link`.

**Configuration**
Wheel odometry node configuration is held in
`ros2_ws/src/jetbot_base/config/wheel_odom.yaml` and includes:
```yaml
ticks_per_rev: 1920.0
wheel_radius_m: 0.0325
wheel_base_m: 0.120

left_sign: 1
right_sign: 1

publish_tf: false
```
Pose and twist covariance diagonals are also configurable for EKF fusion.

### 12.3 Build the ROS 2 Workspace and Launch Nodes

Inside the `ros-foxy-jetbot-dev` container run:
``` bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Launch the wheel odometry node:
```bash
ros2 launch jetbot_base wheel_odom.launch.py \
  params_file:=/ros2_ws/src/jetbot_base/config/wheel_odom.yaml
```

Launch the motor driver node:
```bash
ros2 launch jetbot_base jetbot_motor_driver.launch.py
```

### 12.4 Diagnostics & Debugging
Check diagnostic via `ros2 topic echo /diagnostics` and look for:
* `frames_ok`
* `crc_fail`
* `seq_jumps`
* `last_packet_age_s`

Switch Teensy to JSON mode for manual debugging:
```bash
echo "MODE JSON" > /dev/ttyTHS1
cat /dev/ttyTHS1
```

Switch back:
```bash
echo "MODE BIN" > /dev/ttyTHS1
```

---

## 13. Continued Integrations
The current setup is designed to integrate cleanly with:
* `robot_localization` with extended Kalman filter (EKF)
* IMU fusion from the Intel Realsense D435i integrated IMU
* Nav2/ SLAM toolchains
* ros2_control migration

---

**Author:** Zach Swanson  
**License:** Apache-2.0  
**Project:** JetBot-SLAM (ROS 2 + RealSense + Teensy)  
**Platform:** Jetson Nano 4 GB (JetPack 4.6.1 / L4T 32.7.1)