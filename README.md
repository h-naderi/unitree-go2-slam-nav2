# Unitree-Go2 Robot: SLAM and Nav2 Demos

*By: Hossein Naderi*

Welcome to the **Unitree-Go2 Robot SLAM and Nav2** repository! This project demonstrates the integration of SLAM (Simultaneous Localization and Mapping) and autonomous navigation using Nav2 on the Unitree-Go2 robot. The repository includes demonstration videos, source code, and comprehensive documentation to assist in replicating and building upon the showcased capabilities.&#8203;:contentReference[oaicite:0]{index=0}

---

## Table of Contents

1. [Overview](#overview)
2. [Demo Videos](#demo-videos)
3. [Project Structure](#project-structure)
4 [Workspace Setup](#workspace-setup)
5. [Launch Files](#launch-files)
6. [Detailed Project Explanations](#detailed-project-explanations)

---

## Overview

- **Robot Platform**: [Unitree-Go2](https://www.unitree.com/products/go2)
- **SLAM**: ROS 2 RTAB-Map
- **Navigation**: ROS 2 Nav2

### Key Features

- **Real-time mapping** in indoor and outdoor environments
- **Obstacle avoidance** using Nav2 and sensor data
- **Sensor fusion** combining RGB Depth Camera with LiDAR point cloud
- **Autonomous path planning** to specified goals
- **Facial recognition** capabilities for identifying individuals during navigation

---

## Demo Videos

1. **SLAM Demo Video**  
   *Demonstration of integrated SLAM with Unitree-Go2 using RTAB-Map in ROS2.*

   [![SLAM Demo Video](https://github.com/user-attachments/assets/99b8062b-3fb6-4a19-bc97-b03568393300)](https://github.com/user-attachments/assets/99b8062b-3fb6-4a19-bc97-b03568393300)

2. **Autonomous Navigation Demo Video**  
   *Demonstration of integrated Autonomous Navigation with Unitree-Go2 using Nav2 Stack in ROS2.*

   [![Autonomous Navigation Demo Video](https://github.com/user-attachments/assets/3f4abc8c-d612-4665-a894-e6b635843b2e)](https://github.com/user-attachments/assets/3f4abc8c-d612-4665-a894-e6b635843b2e)



## Dependencies



- **Unitree SDK**: [https://github.com/unitreerobotics/unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)
- **LiDAR SDK**: [https://github.com/RoboSense-LiDAR/rslidar_sdk](https://github.com/RoboSense-LiDAR/rslidar_sdk)
- **LiDAR Messages**: [https://github.com/RoboSense-LiDAR/rslidar_msg](https://github.com/RoboSense-LiDAR/rslidar_msg)

:contentReference[oaicite:17]{index=17}&#8203;:contentReference[oaicite:18]{index=18}

---

## Workspace Setup



1. **Create your workspace:**
   ```bash
   mkdir -p ~/ws/src
   cd ~/ws/src
   ```

2. **Clone this repository:**
   ```bash
   git clone https://github.com/h-naderi/unitree-go2-slam-nav2.git

   ```
3. **Navigate to the repository:**

   ```bash
   cd unitree-go2-slam-nav2
   ```

4. **Import dependencies using `vcs`:**

   ```bash
   vcs import < dependencies.repo
   ```   
5. **Install dependencies and build the workspace:**

   ```bash
   cd ~/ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build
   ```

6. **Source the workspace:**

   ```bash
   source ~/ws/install/setup.bash
   ```

---

## Launch Files

1. **Launch the Intel RealSense camera node:**
    ```sh
    ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_infra1:=true enable_infra2:=true enable_sync:=true
    ```

2. **Launch everything on the robot for mapping with the LiDAR only:**
    ```sh
    ros2 launch go2_slam_nav mapping.launch.py
    ```

3. **Launch the robot with the RealSense camera and the LiDAR:**
    ```sh
    ros2 launch go2_slam_nav mapping_camera.launch.py
    ```
4. **Launch the navigation:**
    ```sh
    ros2 launch go2_slam_nav nav.launch.py
    ```

5. **Launch the exploration node only:**
    ```sh
    ros2 launch frontier frontier_update.launch.xml
    ```



Feel free to reach out if you have any questions or need further assistance with setting up the project.

## Detailed Project Explanations

For comprehensive explanations and insights into each project component, please refer to my portfolio:

- **SLAM Project**: [SLAM Project](https://h-naderi.github.io/projects/1-slam)
- **Autonomous Navigation**: [Navigation Project](https://h-naderi.github.io/projects/3-auto-nav-and-exploration)

