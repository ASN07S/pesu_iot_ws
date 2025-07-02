# 🤖 Autonomous Indoor Navigation using Cartographer SLAM | ROS 2 Humble

This project showcases a 4-wheeled indoor robot developed using **ROS 2 Humble** for autonomous indoor navigation.  
It features:

- 📍 **Real-time 2D SLAM** using [Google Cartographer](https://google-cartographer.readthedocs.io/)
- 🧭 Autonomous path planning using the **Nav2 stack**
- 🧠 Trajectory control via the **Vector Pursuit Controller**
- 🌐 Real-time LiDAR scanning using the **YDLidar X4 Pro**
- 🛠️ Motor control with **Arduino UNO** + **L298N Motor Driver**

The system was tested successfully in simulation, demonstrating full autonomous exploration.  
> ⚠️ On the real robot, full autonomy was **partially implemented** due to hardware tuning and control issues.

Developed as part of a **2025 Internship Project at PES University**.

---

## 🚀 Features

- 🗺️ Real-time 2D SLAM using Cartographer  
- 🤖 Full Nav2-based autonomous navigation  
- 🌈 Smooth trajectory following with **Vector Pursuit**  
- 📡 YDLidar X4 Pro laser scanner integration  
- 🧩 Arduino UNO + Serial Motor Driver control  
- 🧪 Simulation + Real-world Testing  
- 🛠️ CAD + demo video included  

---
<img src="media/cad_front_view.png" alt="Our UGV Model" width="600"/>
<img src="media/cad_side_view.png" alt="Our UGV Model" width="600"/>

---

## 💻 Requirements

### Software
- Ubuntu 22.04
- ROS 2 Humble
- Python 3
- Arduino IDE

### Hardware
- 🔌 Arduino UNO  
- ⚙️ L298N Motor Driver  
- 📡 YDLidar X4 Pro  
- 🔋 Power supply  
- 💻 Laptop running Ubuntu 22.04 + ROS 2 Humble  
- 🍓 Raspberry Pi 4 (optional for edge deployment)

---

## 🧠 Vector Pursuit Controller

This project integrates the [**Vector Pursuit Controller**](https://github.com/blackcoffeerobotics/vector_pursuit_controller) plugin into the **ROS 2 Nav2 stack** to enable smooth and accurate path tracking.

> The controller tracks path orientation in a geometrically meaningful way and is an efficient, low-latency, and lightweight alternative to traditional trajectory-following algorithms.

📢 **Credits**:
- 🧪 The **Vector Pursuit Controller** is developed and maintained by [**Black Coffee Robotics**](https://github.com/blackcoffeerobotics).
- 🏫 Special thanks to **PES University** for providing **hardware access**, **lab facilities**, and **technical mentorship** throughout the course of this internship project.


## 📦 Get Started

## Install ROS 2 Dependencies
- sudo apt update
- sudo apt install ros-humble-cartographer \
  ros-humble-navigation2 \ ros-humble-nav2-bringup \
  ros-humble-vector-pursuit-controller \
  ros-humble-rviz2

### 1. Clone the Repo

```bash
git clone https://github.com/ASN07S/pesu_iot_ws.git
cd pesu_iot_ws
colcon build --symlink-install
source install/setup.bash

### Launch Components for mapping Real_UGV
1.Run YDLidar Driver
  ros2 launch ydlidar_ros2_driver ydlidar_launch.py
2.Launch SLAM
  ros2 launch ydlidar_ros2_driver ydlidar_cartographer_launch.py
3.Save Map from Cartographer SLAM
  ros2 run nav2_map_server map_saver_cli -f ~/your_map

### Launch Components for Navigation Real_UGV
1.Run YDLidar Driver
  ros2 launch ydlidar_ros2_driver ydlidar_launch.py
2. Start Vector Pursuit Navigation
  ros2 launch vector_pursuit_navigation pesu_bot.py
3. Start Arduino Serial Bridge
  ros2 run nav2_arduino_bringup arduino_odometry_node
