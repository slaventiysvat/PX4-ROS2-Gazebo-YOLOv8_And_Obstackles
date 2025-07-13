# PX4-ROS2-Gazebo-YOLOv8

A full-stack drone simulation pipeline with object detection and autonomous navigation.
The system includes **PX4 Autopilot**, **ROS 2 Jazzy**, **Gazebo Ionic**, and **YOLOv8** for object detection.
Simulated using `gz_x500_depth` quadcopter model with forward-facing camera.

---

## ✨ Features

* Offboard control using ROS 2 (`human_follower.py`)
* PX4 SITL simulation with Gazebo Garden / Ionic
* Real-time object detection using YOLOv8
* Optional integration with:

  * Depth camera (`depth_image_proc`)
  * MAVSDK
  * RTAB-Map + Nav2 (local planner)
* Custom SDF models for human/person, cars, wires, buildings, and trees

---

## 🗂 Directory Structure

```bash
~/PX4-Autopilot                # PX4 firmware (v1.15.0+)
~/ws_offboard_control          # ROS 2 workspace for human_follower and control
└── src/
    ├── px4_ros_com
    ├── px4_msgs
    ├── sptools                # Custom ROS 2 package
        ├── launch/
        ├── scripts/
        ├── config/
```

---

## 🛠️ Installation

Tested on **Ubuntu 24.04 LTS**

### 1. Install Gazebo Ionic

Follow official guide: [https://gazebosim.org/docs/latest/install\_ubuntu/](https://gazebosim.org/docs/latest/install_ubuntu/)

---

### 2. Install ROS 2 Jazzy

Install from source (recommended):
[https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Development-Setup.html](https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Development-Setup.html)

---

### 3. Clone Repositories

```bash
# ROS 2 workspace
mkdir -p ~/ws_offboard_control/src && cd ~/ws_offboard_control/src
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/PX4/px4_msgs.git
# custom repo (if private, use SSH)
https://github.com/slaventiysvat/PX4-ROS2-Gazebo-YOLOv8_And_Obstackles
```

```bash
# PX4 Autopilot
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot/
bash Tools/setup/ubuntu.sh
make px4_sitl
```

---

### 4. Python Setup

```bash
python3 -m venv ~/px4-venv
source ~/px4-venv/bin/activate

pip install -U pip
pip install mavsdk aioconsole pygame numpy opencv-python ultralytics
```

---

### 5. Build ROS 2 Workspace

```bash
cd ~/ws_offboard_control
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

---

### 6. Add to `~/.bashrc`

```bash
# ROS & Gazebo
source ~/ws_offboard_control/install/setup.bash
export GZ_VERSION=ionic
export GZ_SIM_RESOURCE_PATH=$HOME/.gz/sim:$HOME/PX4-Autopilot/Tools/simulation/gz/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gz-sim9/plugins
export GZ_CONFIG_PATH=/usr/share/gz
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/local/lib:$LD_LIBRARY_PATH

# GPU Rendering
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export LIBGL_KOPPER_DRI2=1
export QT_OPENGL=desktop

# ROS middleware
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Build aliases
alias colcon_safe='RMW_IMPLEMENTATION=rmw_cyclonedds_cpp MAKEFLAGS="-j4" colcon build --cmake-clean-cache --cmake-args -DBUILD_TESTING=OFF'
```

---

## 🎮 Run Simulation

> Run each terminal in a separate window or use tmux

**Terminal 1 — MAVROS (if used):**

```bash
ros2 run mavros mavros_node \
  --ros-args \
  -p fcu_url:=udp://:14540@localhost:14557 \
  -p config_yaml:=/home/<user>/ws_offboard_control/src/sptools/config/mavros_config.yaml
```

**Terminal 2 — QGroundControl (optional):**

```bash
./QGroundControl.AppImage
```

**Terminal 3 — PX4 + Gazebo:**

```bash
cd ~/PX4-Autopilot
PX4_GZ_LOG_LEVEL=debug PX4_LOCKSTEP=0 \
PX4_SYS_AUTOSTART=4002 \
PX4_SIM_MODEL=gz_x500_depth \
PX4_GZ_MODEL_POSE="-4.78,7.54,0,0,0,-0.7" \
PX4_SIMULATOR=GZ \
PX4_GZ_WORLD=forest \
./build/px4_sitl_default/bin/px4
```

**Terminal 4 — RViz2:**

```bash
rviz2
# Choose image and depth topics
```

**Terminal 5 — Offboard control + Object detection:**

```bash
source ~/px4-venv/bin/activate
cd ~/ws_offboard_control
ros2 launch sptools human_follower.launch.py
```

Drone should take off, rotate, detect a person, and fly toward them.

---

## 🔧 Additional Notes

* Copy models to: `~/.gz/models`
* Copy `default.sdf, forest.sdf, ` to:
  `~/PX4-Autopilot/Tools/simulation/gz/worlds/`
* Adjust camera pose:
  Edit: `x500_depth/model.sdf` → line 9:

```xml
<!-- before -->
<pose>.12 .03 .242 0 0 0</pose>

<!-- after -->
<pose>.15 .029 .21 0 0.7854 0</pose>
```

---

## 📚 Acknowledgements

* [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)
* [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics)
* [Gazebo Sim](https://gazebosim.org/)
* [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)

---

## 📩 Contact

For questions or contributions, please open an issue or contact \[Sviatoslav Starokozhev] at \[[slaventiysvat@gmail.com](mailto:slaventiysvat@gmail.com)].

