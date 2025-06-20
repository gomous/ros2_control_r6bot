# 🤖 6-DOF Robotic Arm Controller using ROS 2 + Joystick Teleoperation

This project demonstrates a complete pipeline for simulating and teleoperating a 6-DOF robotic arm using [`ros2_control`](https://control.ros.org/), custom controllers, and a gamepad (e.g., EvoFox Elite X / Xbox controller) in **ROS 2 Jazzy**.

![Demo](Output.gif)

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

### 3. Build the workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

---

### 4. Launch the controller + RViz

```bash
ros2 launch ros2_control_demo_example_7 r6bot_controller.launch.py
```

---

### 5. Run the joystick node

```bash
ros2 run joy joy_node
```

> You must move the joystick to trigger /joy messages.

---

### 6. Run the teleop controller

```bash
ros2 run ros2_control_demo_example_7 arm_teleop.py
```

Now move your joysticks to move the arm!

---

## 🎮 Controls (EvoFox / Xbox Style)

| Control Stick     | Joint Axis           |
|------------------|----------------------|
| Left Stick X      | Joint 1              |
| Left Stick Y      | Joint 2              |
| Right Stick X     | Joint 3              |
| Right Stick Y     | Joint 4              |
| L2/R2 (Triggers)  | Joint 5, Joint 6     |

> You can modify mappings in [`arm_teleop.py`](src/ros2_control_demo_example_7/scripts/arm_teleop.py)

---

## 🧪 Test Joint States

To confirm the robot is moving:

```bash
ros2 topic echo /joint_states
```

You should see the joint positions update.

---

## 📽️ Demo

![Demo](output.gif)

<!-- > Place your GIF here: `output.gif`  
> To create a GIF: record screen with `peek` or `OBS`, convert to GIF using ffmpeg or [ezgif](https://ezgif.com/) -->

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

- Built using [ros2_control](https://control.ros.org)
- Joystick via ROS `joy` package

---

## 📜 License

Apache-2.0 License  
© 2025 Mousum Gogoi