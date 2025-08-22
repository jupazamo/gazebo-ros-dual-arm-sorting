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
---

### `xarm2\_random\_placer.py`

This node makes **xArm #2** scan the table, detect four colored cubes, and place each one into a randomly assigned (non‑repeating) target zone. It couples **OpenCV** color segmentation with **MoveIt** motion execution.

**What it does (high level)**

1. **Phase 1 – Scan:** subscribes to the RGB camera (`sensor_msgs/CompressedImage`), segments red/blue/green/yellow regions, and detects one rectangle per color.
2. **Phase 2 – Execute:** randomly permutes four destination zones and, for each detected color:

   * converts pixel center → table millimeters
   * transforms table → robot base
   * executes a pick at a safe approach, grasps, lifts, and places at the assigned zone
   * overlays a **single highlighted bounding box** on the live camera for the cube currently being moved.

**Parameters (ROS)**

| Name            | Type      |                       Default | Meaning                                                                    |
| --------------- | --------- | ----------------------------: | -------------------------------------------------------------------------- |
| `~namespace`    | string    |                            "" | Prefix for MoveIt groups (if you run multi-robot with separate namespaces) |
| `~dof`          | int       |                           `6` | xArm DoF (6 or 7)                                                          |
| `~camera_topic` | string    | `camera/image_raw/compressed` | Compressed RGB topic to subscribe                                          |
| `~base_tf`      | float\[3] |                     `[0,0,0]` | Rigid transform **table→robot base**: dx \[mm], dy \[mm], dθ \[deg]        |
| `~safe_z`       | float     |                          `50` | Safe Z (mm) for approach/retreat                                           |
| `~grab_z`       | float     |                       `-67.5` | Pick Z (mm), relative to robot base frame used by MoveIt                   |
| `~robot_type2`  | string    |                        `xarm` | Prefix for MoveIt group names (`xarm6`, `xarm_gripper`)                    |

**Internal constants you can tune in code:**

* `COLOR_DICT` (HSV thresholds)
* `TARGET_ZONES` (mm coordinates of the four mats)
* `ZONE_RANDOM_OFFSET` (random jitter inside a zone)
* `MAX_BLOCK_AREA_PX` (filters out large contours such as mats)

**Topics & I/O**

* **Subscribe:** `~camera_topic` (`sensor_msgs/CompressedImage`)
* **MoveIt groups:** `xarm6` (or namespaced) and `xarm_gripper`
* **No custom publishers** (visual feedback is via OpenCV window and optional MP4 recording)

**Key components in the script**

**1) Vision**

* `get_recognition_rects(frame, lower, upper)`

  * Gaussian blur → HSV → erosion → inRange → contours → `minAreaRect`
  * filters tiny contours and too‑large areas, returns a list of rectangles and a visualization frame
* **Highlighting in UI**

  * `draw_minarearect` draws a labeled box **only** for the cube currently being moved
  * Live display handled by `Display.show()` with optional recording

**2) Pixel → millimeters → robot base**

* `rect_to_xy_mm(rect)` converts the **pixel center** of the rectangle to **table mm** using a calibrated affine mapping
* `mesa_to_robot(x_mm, y_mm, dx, dy, dθ)` rotates and translates table coordinates to the robot base using `~base_tf`

**3) Motion execution**

* `XArmCtrl.moveto(x, y, z)` sets a pose goal (keeping current orientation), MoveIt plans and executes
* `GripperCtrl.open()/close()` actuates the named targets `open` / `close`
* `MotionThread` consumes tasks from a queue: pick at source (approach→grasp→lift), place at destination (approach→open)

**4) Flow (main loop)**

1. **Detection phase** until all 4 colors are located.
2. Randomly assign the four `TARGET_ZONES`.
3. For each color:

   * compute source mm, transform to robot
   * show a single labeled bounding box for context
   * enqueue a pick‑and‑place task, then visually **track the same color** during motion by selecting the nearest current rect to the original center
4. After all moves, send the arm to `home` and exit.


**Coordinate notes & calibration**

* The `rect_to_xy_mm` mapping constants (466, 552, 900/460, offsets) were calibrated for this scene; if you change camera intrinsics, table pose, or image size, recalibrate these values.
* Use `~base_tf` when your **table origin** is not perfectly aligned with the robot base frame.

## Example output (abridged)

```
[INFO] FASE 1: Escaneando la mesa...
  ✓ Bloque 'red' localizado.
  ✓ Bloque 'blue' localizado.
  ✓ Bloque 'green' localizado.
  ✓ Bloque 'yellow' localizado.
FASE 2: Generando y ejecutando el plan...
Recogiendo de (Mesa: 620.4, -210.7)
Soltando en (Mesa: 635.0, -245.0)
...
✓ Tarea completada: 4 bloques redistribuidos aleatoriamente.
```

---

## 📸 Results

![Simulation Example](docs/simulation_example.png)

## 🙌 Acknowledgements

* **UFactory** for the original `xarm_ros` package
* ROS and Gazebo communities for open-source tools
