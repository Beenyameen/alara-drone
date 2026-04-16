# alara-drone

```bash
docker compose up --build
```




# Pi Setup Commands

echo "%sudo ALL=(ALL) NOPASSWD: ALL" | sudo tee /etc/sudoers.d/99-nopasswd

sudo systemctl mask systemd-networkd-wait-online.service

sudo apt update && sudo apt full-upgrade -y && sudo apt autoremove -y




sudo sed -i 's/console=serial0,115200 //g' /boot/firmware/cmdline.txt

sudo sed -i 's/$/ usbcore.usbfs_memory_mb=128/' /boot/firmware/cmdline.txt

sudo apt install python3-gpiozero python3-lgpio python3-venv python3-dev python3-opencv cmake build-essential -y

sudo groupadd -f gpio

sudo usermod -aG dialout,gpio $USER




python3 -m venv venv

source venv/bin/activate

pip install pymavlink pyserial pyzmq pyorbbecsdk2 numpy




git clone https://github.com/orbbec/pyorbbecsdk.git

cd pyorbbecsdk

sudo bash ./scripts/env_setup/install_udev_rules.sh

sudo udevadm control --reload-rules && sudo udevadm trigger




# Pi Services

sudo nano /etc/systemd/system/ip.service

sudo nano /etc/systemd/system/camera.service

sudo nano /etc/systemd/system/geiger.service

sudo nano /etc/systemd/system/fc.service


sudo systemctl daemon-reload

sudo systemctl enable ip.service

sudo systemctl enable camera.service

sudo systemctl enable geiger.service

sudo systemctl enable fc.service

# dronesim

This folder is a ROS 2 workspace for a minimal indoor drone simulator. The only part that matters here is the package under `dronesim/indoor_drone_sim`.

## Overview

`indoor_drone_sim` provides a small simulation stack for indoor drone testing. It is designed for quick validation of:

- manual motion control through `/cmd_vel`
- odometry and TF publishing through `/odom` and `odom -> base_link`
- depth camera data on `/depth/image_raw`, `/depth/image_viz`, `/depth/camera_info`, and `/depth/points`
- radiation sensing through `/geiger_count`
- world visualization markers through `/world_geometry`
- live radiation grid mapping with `radmapper_node`

## Package Layout

The relevant package lives in `dronesim/indoor_drone_sim` and includes:

- ROS 2 nodes for the simulator world, server, mapper, and toucher logic
- launch files for running the simulator with or without RViz
- RViz configuration files
- package metadata and tests

## Requirements

- ROS 2 Jazzy
- Python 3
- A ROS 2 build tool such as `colcon`

The package declares runtime dependencies on common ROS 2 libraries including `rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `visualization_msgs`, `tf2_ros`, `tf_transformations`, `launch`, and `launch_ros`.

## Build

From the workspace root:

```bash
cd dronesim
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

If you are using an existing shell with ROS already sourced, you only need to run the build and then source the workspace overlay.

## Run

Launch the simulator with RViz:

```bash
ros2 launch indoor_drone_sim sim_with_slam.launch.py
```

Launch the simulator without RViz:

```bash
ros2 launch indoor_drone_sim sim.launch.py
```

Both launch files start the core simulator world and server nodes. The two important differences are:

1. `sim_with_slam.launch.py` starts RViz and lets you choose the RViz config through `use_rviz` and `rviz_config_file`. **Deprecated, horrible performance.**
2. `sim.launch.py` also starts `toucher_node` and forces the world node to run in a headless GPU mode with `run_without_rviz=True` and `camera_backend=gpu`. `sim_with_slam.launch.py` leaves those choices out.

The package also exposes executable nodes directly through ROS 2:

- `sim_world_node`
- `sim_server_node`
- `radmapper_node`
- `radmapper_demo`
- `toucher_node`
