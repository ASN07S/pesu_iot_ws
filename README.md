# 🤖 Autonomous Indoor Navigation using Cartographer SLAM | ROS 2 Humble

This project showcases a 4-wheeled indoor robot developed using **ROS 2 Humble** for autonomous navigation. It features **real-time 2D SLAM** using **Google Cartographer**, autonomous path planning with the **Nav2 stack**, and environmental sensing via a **YDLidar X4 Pro**. Motion control is handled by an **Arduino UNO** with an **L298N motor driver**, using a **Vector Pursuit controller** for trajectory following.

The system was successfully implemented and tested in **simulation**, demonstrating complete autonomous exploration and navigation. While the robot was able to perform **real-time mapping on the physical hardware**, full autonomous navigation on the real robot was **partially implemented due to hardware tuning and control challenges**.

Developed as part of a **2025 Internship Project at PES University**.

---

## 📌 Features

- 🗺️ 2D SLAM with Cartographer
- 🧭 Autonomous navigation with Nav2
- 🔦 Real-time LiDAR scanning with YDLidar X4 Pro
- 🌀 Smooth trajectory tracking using Vector Pursuit
- 🛞 Serial motor control with Arduino UNO
- 🎮 Simulation + partial real-world testing
- 🎥 CAD + demo video included

---

## 📋 Requirements

### Software
- **Ubuntu 22.04**
- **ROS 2 Humble**
- Python 3
- Arduino IDE

### Hardware
- 🧠 Arduino UNO
- ⚡ L298N Motor Driver
- 🔦 YDLidar X4 Pro
- 🔋 Power supply
- 💻 Laptop running Ubuntu 22.04 + ROS 2 Humble
- 💻 Raspberry pi 4 running Ubuntu 22.04 + ROS 2 Humble

---

## 🧭 Vector Pursuit Controller

This project integrates the **[Vector Pursuit controller](https://github.com/blackcoffeerobotics/vector_pursuit_controller)** into the **ROS 2 Nav2 stack** to enable smooth and accurate path tracking. The controller tracks path orientation in a geometrically meaningful way and serves as an improved alternative to the traditional Pure Pursuit algorithm. It's efficient, low-latency, and designed for embedded systems.

> We used this controller to enhance navigation accuracy during simulation. On the real robot, it was partially tested and showed stable path alignment performance.

### 🎥 Demo Video

<video width="100%" controls>
  <source src="media/sim_navigation_demo.mp4" type="video/mp4">
  
</video>

---

## 🚀 Get Started

### 1. Clone the Repo

```bash
git clone https://github.com/ASN07S/pesu-iot-indoor-nav.git
cd pesu-iot-indoor-nav
```

### 2. Install ROS 2 Dependencies

```bash
sudo apt update
sudo apt install \
  ros-humble-cartographer \
  ros-humble-navigation2 \
  ros-humble-serial-driver \
  ros-humble-rviz2 \
  ros-humble-vector-pursuit-controller
```

### 3. Build the Workspace

```bash
colcon build
source install/setup.bash
```

### 4. Launch Components

#### Run YDLidar Driver

```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

#### Launch SLAM

```bash
ros2 launch ydlidar_ros2_driver ydlidar_cartographer.launch.py
```

#### Start Nav2

```bash
ros2 launch vector_pursuit_controller pesu_bot.py
```

#### Start Arduino Serial Bridge

```bash
ros2 run nav2_arduino_bringup arduino_bridge
```

---

## 🗂️ Workspace Structure

```
pesu_iot_ws/
├── src/
│   ├── vector_pursuit_controller/
│   ├── ydlidar_ros2_driver/
│   ├── ros2_serial/
├── media/
│   ├── cad_front_view.png
│   ├── cad_side_view.png
│   ├── real_ugv_slam.png
│   └── sim_navigation_demo
├── README.md
```

---

## 📚 Credits & Acknowledgments

- 🔄 **Vector Pursuit Controller**:  
  Integrated from [Black Coffee Robotics](https://github.com/blackcoffeerobotics/vector_pursuit_controller).  
  Licensed under MIT. All credit to the original authors.

- 🗺️ Cartographer & Nav2 – open source ROS 2 community

- 👨‍🏫 PES University – for mentorship and hardware access
