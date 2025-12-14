#!/usr/bin/env python3
"""
Test Launch File for Yahboomcar SDF Model
Tests the modified four_diff.sdf file based on MicroROS.urdf structure
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_yahboomcar = get_package_share_directory('yahboomcar_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Paths
    sdf_file = os.path.join(pkg_yahboomcar, 'urdf', 'four_diff.sdf')
    world_path = os.path.join(pkg_gazebo_ros, 'worlds', 'empty.world')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='true',
        description='Verbose output'
    )

    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial X position'
    )

    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial Y position'
    )

    z_pose_arg = DeclareLaunchArgument(
        'z_pose',
        default_value='0.1',
        description='Initial Z position'
    )

    # Launch Gazebo server
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'verbose': LaunchConfiguration('verbose'),
            'pause': 'false'
        }.items()
    )

    # Launch Gazebo client
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={
            'verbose': LaunchConfiguration('verbose')
        }.items()
    )

    # Robot state publisher (reads SDF and publishes robot_description)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': open(sdf_file).read()
        }]
    )

    # Spawn robot entity in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_yahboomcar',
        output='screen',
        arguments=[
            '-entity', 'yahboomcar',
            '-file', sdf_file,
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', LaunchConfiguration('z_pose'),
            '-timeout', '60.0'
        ]
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        verbose_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,

        # Launch Gazebo
        gzserver_launch,
        gzclient_launch,

        # Robot nodes
        robot_state_publisher_node,
        spawn_entity_node,
    ])
