# 🤖 6-DOF Robotic Arm Controller using ROS 2 + Joystick Teleoperation

This project demonstrates a complete pipeline for simulating and teleoperating a 6-DOF robotic arm using [`ros2_control`](https://control.ros.org/), custom controllers, and a gamepad (e.g., EvoFox Elite X / Xbox controller) in **ROS 2 Jazzy**.



---

## 📦 Features

- Custom `ros2_control` hardware + controller plugin
- Teleoperation using joystick via `joy` package
- Position control of 6 joints
- Simulated joint motion + RViz visualization
- Fully modular and open for extension

---

## 🚀 Quick Start

### 1. Clone the repo

```bash
git clone https://github.com/gomous/ros2_control_r6bot.git
cd ros2_control_r6bot
```

### 2. Install dependencies

Install ROS 2 Jazzy and required packages:

```bash
sudo apt update
sudo apt install ros-jazzy-ros2-control ros-jazzy-controller-manager ros-jazzy-joint-state-broadcaster ros-jazzy-joy
```

---

## 🚧 Docker-Based Setup (Optional)

### 1. Build the Docker Image

```bash
sudo docker build -t 6dofarm:latest .
```
Make sure Your DISPLAY is forwarded correctly:
```echo $DISPLAY
xhost +local:docker
```

### 2. Run the Container

```bash
sudo docker run -it \
  --network=host \
  --ipc=host \
  -v /home/mousum/docker_files:/my_docker_files \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device=/dev/dri \
  --env="QT_X11_NO_MITSHM=1" \
  --env="DISPLAY=$DISPLAY" \
  --env="XDG_RUNTIME_DIR=/tmp/runtime-root" \
  --privileged \
  6dofarm:latest \
  bash
```

---

## 🚀 Launch Instructions

### 1. Build the workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch the Controller + RViz (Terminal 1)

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ros2_control_demo_example_7 r6bot_controller.launch.py
```

### 3. Start Joystick Driver (Terminal 2)

```bash
source /opt/ros/jazzy/setup.bash
ros2 run joy joy_node
```

> You must move the joystick to trigger `/joy` messages.

### 4. Run Teleop Controller (Terminal 3)

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ros2_control_demo_example_7 arm_teleop.py
```

Now move your joysticks to control the robot arm!

---

## 🎮 Controls (EvoFox / Xbox Style)

| Control Stick    | Joint Axis       |
| ---------------- | ---------------- |
| Left Stick X     | Joint 1          |
| Left Stick Y     | Joint 2          |
| Right Stick X    | Joint 3          |
| Right Stick Y    | Joint 4          |
| L2/R2 (Triggers) | Joint 5, Joint 6 |

> You can modify mappings in [`arm_teleop.py`](src/ros2_control_demo_example_7/scripts/arm_teleop.py)

---

## 🧪 Test Joint States

To confirm the robot is moving:

```bash
ros2 topic echo /joint_states
```

You should see the joint positions update.

---

## 📽 Demo



---

## 📁 Project Structure

```bash
ros2_control_r6bot/
├── robotic_arm_6_dof/
│   ├── hardware/
│   ├── controller/
│   ├── scripts/
│   │   └── arm_teleop.py
│   ├── urdf/
│   └── launch/
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 🙌 Credits

- Built using [ros2\_control](https://control.ros.org)
- Joystick via ROS `joy` package

---

## 📜 License

Apache-2.0 License\
© 2025 Mousum Gogoi
