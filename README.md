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

## 🐳 Docker-Based Quick Start

### 1. Build the Docker Image

```bash
sudo docker build -t 6dofarm:latest .
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

### 3. Run the System Inside the Container

#### 🖥 Terminal 1: Launch the Controller + RViz

```bash
ros2 launch ros2_control_demo_example_7 r6bot_controller.launch.py
```

#### 🎮 Terminal 2: Start Joystick Driver

```bash
ros2 run joy joy_node
```

> You must move the joystick to trigger `/joy` messages.

#### 🤖 Terminal 3: Run Teleop Controller

```bash
ros2 run ros2_control_demo_example_7 arm_teleop.py
```

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

---

## 📽️ Demo

![Demo](output.gif)

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
