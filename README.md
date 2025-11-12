# JetBot-SLAM Setup Guide (Jetson Nano)

This document details how to flash, configure, and deploy the **JetBot-SLAM** 
environment on a Jetson Nano 4 GB using **JetPack 4.6.1 (L4T 32.7.6)**. These
instructions assume that the Jetson Nano is part of the Waveshare Jetbot AI Kit
and is therefore powered from the kits power distribution board via GPIO. The
instructions include JetBot utilities, Docker + ROS 2 Foxy + RealSense, kernel patching, Teensy setup, and Foxglove visualization.

---

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

## 8. Launch Docker Containers

To easily build both the ROS2 Foxy container (with RealSense topics) and ROS2
Galactic container (Foxglove Bridge). Just run

```bash
cd ~/workspace/jetbot-slam/
docker-compose build
docker-compose up -d
```

You can confirm operation by opening Foxglove Studios on a host computer and
connecting to `ws://<jetbot-addr>:8765`. If both are running you should make a
successful connection and see topics like `/camera/color/image_raw`.

If you want to stop the containers, run

```bash
docker-compose down
```

If you're running into issues, it may be beneficial to remove any orphaned
containers by running

```bash
docker-compose down --remove-orphans
```

To build, launch, or stop the containers independently just append the above 
commands with `ros_foxy_realsense` or `ros_foxy_realsense`, respectively.

### 9.1 Build and Install Teensy Loader CLI
```bash
cd ~/zms_engineering
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

---

**Author:** Zach Swanson  
**Project:** JetBot-SLAM (ROS 2 + RealSense + Teensy)  
**Platform:** Jetson Nano 4 GB (JetPack 4.6.1 / L4T 32.7.1)
