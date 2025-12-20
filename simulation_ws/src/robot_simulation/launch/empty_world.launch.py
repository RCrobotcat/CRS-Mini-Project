#!/usr/bin/env python3
"""
Gazebo Empty World Launch File

Function Description:
    This launch file is responsible for starting the Gazebo simulation environment
    and loading a specified world file.
    It focuses only on loading the world environment and does not include any robot
    spawning logic, following the Single Responsibility Principle.

Main Components:
    - gzserver: Gazebo server, responsible for physics simulation
    - gzclient: Gazebo client (GUI), provides visualization interface

Launch Arguments:
    - world_name: Name of the world file (default: 'empty_world.world')
    - gui: Whether to launch the Gazebo GUI (default: true)
    - verbose: Whether to output detailed logs (default: false)
    - pause: Whether to start the simulation paused (default: false)
    - use_sim_time: Whether to use simulation time (default: true)

Available World Files:
    - empty_world.world: Empty world with only a ground plane
    - turtlebot3_world.world: TurtleBot3 test world
    - turtlebot3_house.world: Indoor house environment
    - turtlebot3_dqn_stage1.world ~ stage4.world: Reinforcement learning training stages
    - turtlebot3_autorace_2020.world: Autonomous driving competition track

Usage Examples:
    # Launch empty world
    ros2 launch robot_simulation empty_world.launch.py

    # Launch a specified world
    ros2 launch robot_simulation empty_world.launch.py world_name:=turtlebot3_world.world

    # Run without GUI (for headless servers or automated testing)
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
    # ========== Path Configuration ==========
    # Get the path of gazebo_ros package for invoking standard Gazebo launch files
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Get the path of the current package (robot_simulation)
    pkg_robot_simulation = get_package_share_directory('robot_simulation')

    # Path to the worlds directory
    worlds_dir = os.path.join(pkg_robot_simulation, 'worlds')

    # ========== Launch Argument Declarations ==========
    # Declare world_name argument: name of the world file to load
    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='2.world',
        description='Gazebo world file name (without path)'
    )

    # Declare gui argument: whether to start Gazebo GUI
    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to start Gazebo GUI client (true/false)'
    )

    # Declare verbose argument: whether to enable verbose logging
    declare_verbose = DeclareLaunchArgument(
        'verbose',
        default_value='true',
        description='Whether to output verbose logs (true/false)'
    )

    # Declare pause argument: whether to start simulation paused
    declare_pause = DeclareLaunchArgument(
        'pause',
        default_value='false',
        description='Whether to start simulation paused (true/false)'
    )

    # Declare use_sim_time argument: whether ROS nodes use simulation time
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Whether to use simulation time (true/false)'
    )

    # ========== Dynamically Construct World File Path ==========
    # Build the full path to the world file based on world_name argument
    world_path = PathJoinSubstitution([
        worlds_dir,
        LaunchConfiguration('world_name')
    ])

    # ========== Gazebo Server Launch ==========
    # Launch gzserver: Gazebo physics simulation engine
    # gzserver is responsible for:
    #   1. Physics engine computation (collision detection, dynamics)
    #   2. Sensor simulation (LiDAR, camera, etc.)
    #   3. Plugin execution
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

    # ========== Gazebo Client Launch ==========
    # Launch gzclient: Gazebo graphical user interface
    # gzclient is responsible for:
    #   1. 3D visualization rendering
    #   2. User interaction (mouse and keyboard)
    #   3. Toolbars and control panels
    # Note: The condition parameter allows GUI to be launched optionally
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('gui')),
        launch_arguments=[
            ('verbose', LaunchConfiguration('verbose')),
        ]
    )

    # ========== Build Launch Description ==========
    return LaunchDescription([
        # Launch argument declarations
        declare_world_name,
        declare_gui,
        declare_verbose,
        declare_pause,
        declare_use_sim_time,

        # Gazebo components
        gzserver_launch,   # Server must start first
        gzclient_launch,   # Then client (if gui=true)
    ])

