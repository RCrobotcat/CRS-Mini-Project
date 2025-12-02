1、如何建图？
运行仿真环境：ros2 launch robot_simulation simulation.launch.py
运行建图算法：ros2 launch robot_nav2 gmapping.launch.py
运行rviz可视化：ros2 launch robot_nav2 slam_laser_rviz.launch.py
运行键盘控制节点：ros2 run teleop_twist_keyboard teleop_twist_keyboard.py 
按键盘控制节点提示控制小车移动扫完整个地图场景后保存地图文件：ros2 launch robot_nav2 map_save.launch.py

2、如何取点？
运行仿真环境：ros2 launch robot_simulation simulation.launch.py
运行代价地图过滤器：ros2 launch  robot_nav2 costmap_filter_info.launch.py
运行导航栈：ros2 launch robot_nav2 navigation.launch.py
运行取点：ros2 run robot_nav2  get_points.py
按照终端提示在rviz中使用2D goal pose分别取起点和终点的坐标，结果会自动保存在csv中

2、在建图和取点完成后：
运行仿真环境：ros2 launch robot_simulation simulation.launch.py
运行代价地图过滤器：ros2 launch  robot_nav2 costmap_filter_info.launch.py
运行导航栈：ros2 launch robot_nav2 navigation.launch.py
运行主程序：ros2 run robot_nav2  go.py