# Gazebo ROS Dual Arm Sorting

A ROS-based simulation project for controlling two robotic arms in Gazebo to perform an automated sorting task. The system detects cube colors, plans grasping motions, and places each cube in its designated location.

## 📌 Overview

This project simulates a **dual robotic arm sorting station** using **ROS** and **Gazebo**, integrating motion planning, perception, and manipulation:

1. One arm simulates a user, placing four differently colored cubes each in a unique zone without repetition.
2. The second arm checks whether the cubes are correctly placed.
3. It repositions incorrectly placed cubes.
4. It organizes remaining cubes by color.

Color detection uses **OpenCV**, and the available camera model is **RealSense D435**.

## ✨ Features

* Dual-arm robot simulation in Gazebo
* Color-based detection with OpenCV
* RealSense D435 camera model
* Grasping and placing with MoveIt
* Predefined sorting positions per color
* ROS-integrated launch files for quick setup

## 📦 Requirements

* ROS Noetic
* Gazebo 11
* MoveIt
* Ubuntu 20.04 LTS

## ⚙️ Installation

```bash
cd ~/catkin_ws/src
git clone https://github.com/jupazamo/gazebo-ros-dual-arm-sorting.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 🚀 Usage

Run in separate terminals:

```bash
roslaunch xarm_gazebo xarm_camera_scene.launch
rosrun xarm_gazebo script_launcher.py
```

### 🖥 Key Scripts

#### **script\_launcher.py**

This script sequentially launches the main simulation control nodes:

1. **xarm2\_random\_placer.py** – Simulates a user-controlled arm randomly placing cubes.
2. **zone\_checker.py** – Verifies if the cubes are correctly positioned in their respective zones.
3. **color\_recognition.py** – Detects cube colors using OpenCV for sorting.

It sets the correct ROS namespace for each process and ensures each step completes before moving to the next.

## 📂 Repository Structure

```
xarm_ros/
 ├── LICENSE
 ├── README.md
 ├── uf_robot_moveit_config/
 ├── xarm6_moveit_config/
 ├── xarm_bringup/
 ├── xarm_description/
 ├── xarm_gripper/
 ├── xarm_msgs/
 ├── xarm_sdk/
 ├── xarm6_gripper_moveit_config/
 ├── xarm_api/
 ├── xarm_controller/
 ├── xarm_gazebo/
 │   ├── CMakeLists.txt
 │   ├── package.xml
 │   ├── launch/
 │   │   └── xarm_camera_scene.launch
 │   ├── scripts/
 │   │   ├── color_recognition.py
 │   │   ├── hsv_inspector.py
 │   │   ├── script_launcher.py
 │   │   ├── xarm2_random_placer.py
 │   │   └── zone_checker.py
 │   └── worlds/
 │       └── xarm_camera_scene.world
 ├── xarm_moveit_servo/
 └── xarm_planner/
```

## 🧠 Key scripts explained

### `script_launcher.py`

A lightweight orchestrator that runs the three main nodes in sequence using the correct ROS namespaces.

**What it does**

* Sets environment variable `ROS_NAMESPACE` before each subprocess to scope topics/services.
* Launches:

  1. `xarm2_random_placer.py` under namespace `xarm2` (simulated user arm that randomly places 4 cubes, no repeated colors).
  2. `zone_checker.py` under namespace `xarm` (validation arm that detects misplacements and fixes them).
  3. `color_recognition.py` under namespace `xarm` (same arm that then sorts remaining center blocks by color).
* Waits for each node to exit (`Popen.wait()`), then starts the next one. This guarantees a clean, staged workflow.

**Why namespaces?**
Two arms are spawned in the scene. Namespacing (`xarm` / `xarm2`) prevents topic collisions between controllers, MoveIt groups, and camera subscribers/publishers.

**Minimum requirements**

* Python 3 (shebang `#!/usr/bin/env python3`)
* ROS environment sourced (`source devel/setup.bash`)
* Executable permission set: `chmod +x xarm_gazebo/scripts/script_launcher.py`
* The following nodes available in your `ROS_PACKAGE_PATH`:

  * `xarm_gazebo/scripts/xarm2_random_placer.py`
  * `xarm_gazebo/scripts/zone_checker.py`
  * `xarm_gazebo/scripts/color_recognition.py`

**How to run**

1. In Terminal A: `roslaunch xarm_gazebo xarm_camera_scene.launch`
2. In Terminal B: `rosrun xarm_gazebo script_launcher.py`

**Command-line overrides**
The script passes `'_robot_type:=xarm'` style private parameters. If you extend to other robot types, adapt:

```python
ROBOT_TYPE = "xarm"
ROBOT_TYPE2 = "xarm"
# ...
subprocess.Popen(['rosrun','xarm_gazebo','xarm2_random_placer.py', f'_robot_type:={ROBOT_TYPE2}'], env=env)
```

**Typical console output**

```
[SCRIPT LAUNCHER] Iniciando xarm2_random_placer.py...
... (placement logs) ...
[SCRIPT LAUNCHER] Finalizado zone_checker.py.
[SCRIPT LAUNCHER] Iniciando zone_checker.py...
... (correction logs) ...
[SCRIPT LAUNCHER] Finalizado zone_checker.py.
[SCRIPT LAUNCHER] Iniciando color_recognition.py...
... (final sorting logs) ...
[SCRIPT LAUNCHER] Finalizado color_recognition.py.
```

**Troubleshooting**

* *`rosrun: command not found`*: ensure ROS is installed and sourced.
* *`[rospack] Error: package 'xarm_gazebo' not found`*: verify the repo path under `~/catkin_ws/src`, then `catkin_make` and re-`source`.
* *Permission denied*: `chmod +x xarm_gazebo/scripts/*.py`.
* *Nodes don’t find the right topics*: confirm `ROS_NAMESPACE` values and that the scene has both arms (`xarm` and `xarm2`).

---

## 📸 Results

![Simulation Example](docs/simulation_example.png)

## 🙌 Acknowledgements

* **UFactory** for the original `xarm_ros` package
* ROS and Gazebo communities for open-source tools
