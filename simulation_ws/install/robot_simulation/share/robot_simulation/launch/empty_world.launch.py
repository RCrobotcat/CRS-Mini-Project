#!/usr/bin/env python3
"""
Gazebo空世界启动Launch文件

功能说明：
    此launch文件负责启动Gazebo仿真环境，加载指定的世界文件。
    本文件专注于世界环境的加载，不包含机器人的生成逻辑，实现了单一职责原则。

主要组件：
    - gzserver: Gazebo服务器端，负责物理仿真计算
    - gzclient: Gazebo客户端（GUI），提供可视化界面

Launch参数：
    - world_name: 世界文件名称 (默认: 'empty_world.world')
    - gui: 是否启动Gazebo GUI (默认: true)
    - verbose: 是否输出详细日志 (默认: false)
    - pause: 启动时是否暂停仿真 (默认: false)
    - use_sim_time: 是否使用仿真时间 (默认: true)

可用的世界文件：
    - empty_world.world: 空白世界，只有地平面
    - turtlebot3_world.world: TurtleBot3测试世界
    - turtlebot3_house.world: 房屋环境
    - turtlebot3_dqn_stage1.world ~ stage4.world: 强化学习训练场景
    - turtlebot3_autorace_2020.world: 自动驾驶比赛场地

使用示例：
    # 启动空白世界
    ros2 launch robot_simulation empty_world.launch.py

    # 启动指定世界
    ros2 launch robot_simulation empty_world.launch.py world_name:=turtlebot3_world.world

    # 无GUI模式（适用于无头服务器或自动化测试）
    ros2 launch robot_simulation empty_world.launch.py gui:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition


def generate_launch_description():
    # ========== 路径配置 ==========
    # 获取gazebo_ros功能包的路径，用于调用Gazebo的标准launch文件
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # 获取当前功能包（robot_simulation）的路径
    pkg_robot_simulation = get_package_share_directory('robot_simulation')

    # worlds目录的路径
    worlds_dir = os.path.join(pkg_robot_simulation, 'worlds')

    # ========== Launch参数声明 ==========
    # 声明world_name参数：指定要加载的世界文件名
    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='empty_world.world',
        description='Gazebo世界文件名称（不含路径）'
    )

    # 声明gui参数：是否启动Gazebo图形界面
    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='是否启动Gazebo GUI客户端（true/false）'
    )

    # 声明verbose参数：是否输出详细的调试信息
    declare_verbose = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='是否输出详细日志信息（true/false）'
    )

    # 声明pause参数：仿真启动时是否暂停
    declare_pause = DeclareLaunchArgument(
        'pause',
        default_value='false',
        description='启动时是否暂停仿真（true/false）'
    )

    # 声明use_sim_time参数：ROS节点是否使用仿真时间
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间（true/false）'
    )

    # ========== 动态构建世界文件路径 ==========
    # 根据world_name参数构建完整的世界文件路径
    world_path = PathJoinSubstitution([
        worlds_dir,
        LaunchConfiguration('world_name')
    ])

    # ========== Gazebo服务器启动 ==========
    # 启动gzserver：Gazebo的物理仿真引擎
    # gzserver负责：
    #   1. 物理引擎计算（碰撞检测、动力学）
    #   2. 传感器仿真（激光雷达、相机等）
    #   3. 插件系统运行
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments=[
            ('world', world_path),
            ('verbose', LaunchConfiguration('verbose')),
            ('pause', LaunchConfiguration('pause')),
        ]
    )

    # ========== Gazebo客户端启动 ==========
    # 启动gzclient：Gazebo的图形用户界面
    # gzclient负责：
    #   1. 3D可视化渲染
    #   2. 用户交互（鼠标、键盘控制）
    #   3. 工具栏和控制面板
    # 注意：condition参数使得可以选择性地启动GUI
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('gui')),
        launch_arguments=[
            ('verbose', LaunchConfiguration('verbose')),
        ]
    )

    # ========== 构建Launch描述 ==========
    return LaunchDescription([
        # Launch参数声明
        declare_world_name,
        declare_gui,
        declare_verbose,
        declare_pause,
        declare_use_sim_time,

        # Gazebo组件启动
        gzserver_launch,  # 必须先启动server
        gzclient_launch,  # 然后启动client（如果gui=true）
    ])
