# Gazebo ROS Dual Arm Sorting

A ROS-based simulation project for controlling two robotic arms in Gazebo to perform an automated sorting task.
The system is capable of detecting cube colors, planning grasping motions, and placing each cube in its designated location.

## 📌 Overview

This project simulates a **dual robotic arm sorting station** using **ROS** and **Gazebo**.
It integrates motion planning, perception, and manipulation to achieve an autonomous sorting workflow:

1. One robotic arm simulates a user, placing four differently colored cubes each in a distinct zone without repetition.
2. The second robotic arm checks whether the cubes have been placed correctly.
3. This same arm repositions any cubes placed incorrectly.
4. Finally, it analyzes the remaining cubes and organizes them by color accordingly.

The project was adapted from **UFactory's xArm ROS packages**, with custom modifications for the sorting application.

## ✨ Features

* Dual-arm robot simulation in Gazebo.
* Color-based cube detection using OpenCV.
* Utilizes a RealSense D435 camera model included in the repository.
* Grasping and placing tasks using MoveIt.
* Predefined sorting positions for each cube color.
* Fully ROS-integrated with launch files for quick setup.

## 📦 Requirements

* **ROS Noetic** (recommended)
* **Gazebo 11**
* **MoveIt** for motion planning
* `xarm_ros` package (included in this repository)
* Ubuntu 20.04 LTS

## ⚙️ Installation

```bash
# Clone the repository inside your ROS workspace
cd ~/catkin_ws/src
git clone https://github.com/jupazamo/gazebo-ros-dual-arm-sorting.git

# Build the workspace
cd ~/catkin_ws
catkin_make

# Source the workspace
source devel/setup.bash
```

## 🚀 Usage

To launch the full simulation:

```bash
roslaunch xarm_gazebo xarm_camera_scene.launch   # Run this in the first terminal
rosrun xarm_gazebo script_launcher.py            # Run this in a separate terminal
```

**Example workflow:**

1. The Gazebo environment loads with two robotic arms and colored cubes.
2. The perception node detects cube colors and positions.
3. The motion planner generates pick-and-place trajectories.
4. Each cube is placed in its designated location.

## 📂 Repository Structure

```
xarm_ros/
 ├── launch/         # Launch files for simulation and control
 ├── scripts/        # Custom scripts for sorting logic
 ├── config/         # MoveIt and robot configuration files
 ├── urdf/           # Robot models
 └── worlds/         # Gazebo world with sorting environment
```

## 📸 Results

![Simulation Example](docs/simulation_example.png)

*Dual-arm sorting station in action.*

## 📜 License

```
Copyright (c) 2018, UFACTORY Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
[... full BSD-3-Clause license text ...]
```
