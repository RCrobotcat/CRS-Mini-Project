## CRS Mini Project – Autonomous Maze Navigation Robot

This repository implements an autonomous maze navigation system based on **ROS 2 + Micro-ROS (ESP32)**.
The robot is capable of **global localization, A* path planning, and fully autonomous navigation** in a known maze with an unknown initial pose.

The project supports **both Gazebo simulation and real-robot execution**, allowing full evaluation without physical hardware.

---

## Repository Structure
```text
CRS-Mini-Project/
│
├── build/                          (Build-related scripts and settings)
├── install/                        (Installed files / build outputs)
├── log/                            (Log directory)
├── gmapping_ws/                    (ROS workspace for gmapping-based map building)
├── imu_ws/                         (ROS workspace related to IMU sensors)
├── simulation_ws/                  (ROS workspace for simulation)
├── yahboomcar_ros2_ws/             (ROS2 workspace)
│   └── yahboomcar_ws/              (Nested ROS workspace)
├── yahboomcar_ws/                  (Another ROS workspace)
│
├── .gitignore                      (Git ignore configuration)
├── .python_history                 (Python command history)
├── .sudo_as_admin_successful       (sudo privilege record file)
├── README.md                       (Project documentation)
├── SET_Camera.py                   (Python script for camera setup)
├── Version.txt                     (Version information file)
├── config_robot.py                 (Robot configuration script)
├── start_Camera_computer.sh        (Script to start the camera-side system)
├── start_agent_computer.sh         (Script to start the agent-side system)
├── yahboomcar_robot2.sdf           (Robot model SDF file)

```

---

## Technical Guidelines (Simulation & Real Robot)

This section provides step-by-step instructions for **both simulation-only evaluation** and **real robot deployment**.

---

### 1. Prerequisites

* Ubuntu 22.04
* ROS 2 Humble
* Gazebo 11
* RViz2
* Nav2
* GMapping

---

## 2. Simulation Workflow (No Physical Robot Required)

### Step 0: Grant Script Permission

```bash
chmod +x gazebo_yahboom.sh
```

### Step 1: Launch Gazebo Simulation

```bash
./gazebo_yahboom.sh
```

(Manual equivalent)

```bash
source /usr/share/gazebo-11/setup.sh
source install/setup.bash
export GAZEBO_MODEL_PATH="${PWD}/src/robot_simulation/models:${PWD}/install/yahboomcar_description/share:${PWD}/src/robot_description/models:${GAZEBO_MODEL_PATH}"
export GAZEBO_RESOURCE_PATH="${PWD}/install/yahboomcar_description/share:${GAZEBO_RESOURCE_PATH}"
ros2 launch robot_simulation simulation.launch.py
```

### Step 2: SLAM Mapping

```bash
ros2 launch robot_nav2 gmapping.launch.py
ros2 launch robot_nav2 slam_laser_rviz.launch.py
ros2 run yahboomcar_ctrl yahboom_keyboard
ros2 launch robot_nav2 map_save.launch.py
```

### Step 3: Select Start & Goal Points

```bash
ros2 launch robot_nav2 navigation.launch.py
ros2 run robot_nav2 get_points.py
```

Use **2D Goal Pose** in RViz to select start and goal points (saved automatically).

### Step 4: Autonomous Navigation

```bash
./gazebo_yahboom.sh
ros2 launch robot_nav2 navigation.launch.py
ros2 run robot_nav2 go.py
```

The robot will localize using **AMCL**, plan paths using **A***, and navigate autonomously.

---

## 3. Real Robot Workflow (Yahboom MicroROS-ESP32)

> This section is for **full system validation using the physical robot**.

### Step 1: Hardware & Environment Initialization

```bash
ls /dev/ttyUSB*
python3 config_robot.py
sh ~/start_agent_computer.sh
ros2 node list
sh ros2_humble.sh
```

### Step 2: Launch Robot & Visualization

```bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py
ros2 launch yahboomcar_nav display_launch.py
```

### Step 3: SLAM Mapping (Real Environment)

```bash
ros2 launch yahboomcar_nav map_gmapping_launch.py
ros2 run yahboomcar_ctrl yahboom_keyboard
ros2 launch yahboomcar_nav save_map_launch.py
```

Default map path:

```text
/home/yahboom/yahboomcar_ws/src/yahboomcar_nav/maps
```

### Step 4: Real-World Navigation

```bash
ros2 launch yahboomcar_nav navigation_dwb_launch.py \
maps:=/home/yahboom/yahboomcar_ws/src/yahboomcar_nav/maps/yahboom_map.yaml
```

The robot will autonomously navigate using the saved map.

---

## Key Highlights

* Micro-ROS bridge between ESP32 and ROS 2
* AMCL global localization (unknown initial pose)
* Nav2 + A* global planning
* Unified workflow for **simulation and real robot**
