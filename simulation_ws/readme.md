# Yahboomcar Gazebo Navigation (ROS 2)

---

## 0. Make Script Executable

```bash
chmod +x gazebo_yahboom.sh
```

---

## 1. Build Map

Start the Gazebo simulation:

```bash
./gazebo_yahboom.sh
```

Equivalent manual commands:

```bash
source /usr/share/gazebo-11/setup.sh
source install/setup.bash
export GAZEBO_MODEL_PATH="${PWD}/src/robot_simulation/models:${PWD}/install/yahboomcar_description/share:${PWD}/src/robot_description/models:${GAZEBO_MODEL_PATH}"
export GAZEBO_RESOURCE_PATH="${PWD}/install/yahboomcar_description/share:${GAZEBO_RESOURCE_PATH}"
ros2 launch robot_simulation simulation.launch.py
```

Run SLAM and visualization:

```bash
ros2 launch robot_nav2 gmapping.launch.py
ros2 launch robot_nav2 slam_laser_rviz.launch.py
```

Run keyboard control:

```bash
ros2 run yahboomcar_ctrl yahboom_keyboard
```

Drive the robot to scan the whole environment, then save the map:

```bash
ros2 launch robot_nav2 map_save.launch.py
```

---

## 2. Get Start and Goal Points

Start the Gazebo simulation:

```bash
./gazebo_yahboom.sh
```

Run navigation components:

```bash
ros2 launch robot_nav2 costmap_filter_info.launch.py
ros2 launch robot_nav2 navigation.launch.py
ros2 run robot_nav2 get_points.py
```

Use **2D Goal Pose** in RViz to select the start and goal points.  
The coordinates will be saved automatically to a CSV file.

---

## 3. Run Navigation

Start the Gazebo simulation:

```bash
./gazebo_yahboom.sh
```

Run navigation:

```bash
ros2 launch robot_nav2 costmap_filter_info.launch.py
ros2 launch robot_nav2 navigation.launch.py
ros2 run robot_nav2 go.py
```

The robot will navigate automatically from the start point to the goal point.

