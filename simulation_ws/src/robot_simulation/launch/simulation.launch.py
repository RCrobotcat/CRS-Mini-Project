#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ========== Path Configuration ==========
    # Get the launch directory path of the current package
    launch_file_dir = os.path.join(
        get_package_share_directory('robot_simulation'),
        'launch'
    )

    # ========== Launch Argument Declarations ==========
    # --- World environment arguments ---
    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='2.world',
        description='Gazebo world file name'
    )

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to start Gazebo GUI'
    )

    declare_verbose = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Whether to output verbose logs'
    )

    declare_pause = DeclareLaunchArgument(
        'pause',
        default_value='false',
        description='Whether to start simulation paused'
    )

    # --- Robot arguments ---
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='robotcar',
        description='Robot name / namespace'
    )

    declare_x_pose = DeclareLaunchArgument(
        'x_pose',
        default_value='1.26',
        description='Initial X position of the robot (meters)'
    )

    declare_y_pose = DeclareLaunchArgument(
        'y_pose',
        default_value='-1.44',
        description='Initial Y position of the robot (meters)'
    )

    declare_z_pose = DeclareLaunchArgument(
        'z_pose',
        default_value='0.05',
        description='Initial Z position of the robot (meters)'
    )

    declare_yaw_pose = DeclareLaunchArgument(
        'yaw_pose',
        default_value='-3.14',
        description='Initial yaw angle of the robot (radians)'
    )

    # --- Common arguments ---
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Whether to use simulation time'
    )

    # ========== Include Sub Launch Files ==========
    # 1. Launch Gazebo world environment
    # Responsible for starting gzserver and gzclient, and loading the world file
    launch_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'empty_world.launch.py')
        ),
        launch_arguments=[
            ('world_name', LaunchConfiguration('world_name')),
            ('gui', LaunchConfiguration('gui')),
            ('verbose', LaunchConfiguration('verbose')),
            ('pause', LaunchConfiguration('pause')),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
        ]
    )

    # 2. Spawn the robot model
    # Responsible for publishing robot description and spawning the robot entity in Gazebo
    launch_robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_spawn.launch.py')
        ),
        launch_arguments=[
            ('robot_name', LaunchConfiguration('robot_name')),
            ('x_pose', LaunchConfiguration('x_pose')),
            ('y_pose', LaunchConfiguration('y_pose')),
            ('z_pose', LaunchConfiguration('z_pose')),
            ('yaw_pose', LaunchConfiguration('yaw_pose')),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
        ]
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # ========== Build Launch Description ==========
    return LaunchDescription([
        # --- Argument declarations ---
        # World environment arguments
        declare_world_name,
        declare_gui,
        declare_verbose,
        declare_pause,

        # Robot arguments
        declare_robot_name,
        declare_x_pose,
        declare_y_pose,
        declare_z_pose,
        declare_yaw_pose,

        # Common arguments
        declare_use_sim_time,

        # --- Launch order ---
        # Note: Although listed in order, the launch system starts nodes in parallel
        # spawn_entity in robot_spawn has a timeout mechanism and will wait for Gazebo to be ready
        launch_empty_world,     # Start the world environment first
        launch_robot_spawn,     # Then spawn the robot
        robot_state_publisher_cmd
    ])

