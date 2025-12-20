#!/usr/bin/env python3
"""
Robot Spawn Launch File

Function Description:
    This launch file is responsible for spawning a robot model in the Gazebo
    simulation environment, including:
    1. Publishing the robot URDF description (via robot_state_publisher)
    2. Spawning the robot entity in Gazebo
    3. Setting the robot's initial pose (position and orientation)

Main Nodes:
    - robot_state_publisher: Publishes robot TF transforms and robot_description
    - spawn_entity.py: Gazebo entity spawning node

Launch Arguments:
    - robot_name: Robot name / prefix (default: 'nanocar')
    - x_pose: Initial X position of the robot (default: 0.0)
    - y_pose: Initial Y position of the robot (default: 0.0)
    - z_pose: Initial Z position of the robot (default: 0.0)
    - yaw_pose: Initial yaw angle / heading (default: 0.0)
    - use_sim_time: Whether to use simulation time (default: true)

Usage Example:
    ros2 launch robot_simulation robot_spawn.launch.py robot_name:=my_robot x_pose:=1.0 y_pose:=2.0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ========== Path Configuration ==========
    # Get the package share directory
    pkg_share = FindPackageShare(package='robot_simulation').find('robot_simulation')

    # Full path to the SDF model file
    sdf_file = os.path.join(
        pkg_share,
        'models/four_diff/model.sdf',
    )

    # ========== Environment Variable Setup ==========
    # Set Gazebo model path so Gazebo can locate robot meshes and resources
    models_dir = os.path.join(pkg_share, 'models')

    # Add yahboomcar_description to the model path
    yahboom_pkg_share = FindPackageShare(package='yahboomcar_description').find('yahboomcar_description')

    # Get the current GAZEBO_MODEL_PATH
    current_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')

    # Combine all paths
    combined_model_path = f"{models_dir}:{yahboom_pkg_share}:{current_model_path}"

    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=combined_model_path
    )

    # ========== Launch Argument Declarations ==========
    # Declare robot_name argument: used to identify the robot in multi-robot simulations
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='mycar',
        description='Robot name used for namespace and entity identification'
    )

    # Declare initial position argument: robot initial X position in Gazebo world
    declare_x_pose = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial X position of the robot (meters)'
    )

    # Declare initial position argument: robot initial Y position in Gazebo world
    declare_y_pose = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial Y position of the robot (meters)'
    )

    # Declare initial position argument: robot initial Z position (height)
    declare_z_pose = DeclareLaunchArgument(
        'z_pose',
        default_value='0.0',
        description='Initial Z position of the robot (meters)'
    )

    # Declare initial orientation argument: robot initial yaw angle (rotation around Z-axis)
    declare_yaw_pose = DeclareLaunchArgument(
        'yaw_pose',
        default_value='0.0',
        description='Initial yaw angle of the robot (radians), 0 means facing east'
    )

    # Declare simulation time argument: whether to use Gazebo simulation time
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Whether to use simulation time (true/false)'
    )

    # ========== Node Definitions ==========
    # spawn_entity node: spawns the robot entity in Gazebo
    # Function: reads the model description from SDF and spawns the robot at a given pose
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            # Entity name used to identify the robot in Gazebo
            '-entity', LaunchConfiguration('robot_name'),
            # Load robot description from SDF file
            '-file', sdf_file,
            # Set initial robot position (X, Y, Z)
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', LaunchConfiguration('z_pose'),
            # Set initial robot orientation (Yaw angle around Z-axis)
            '-Y', LaunchConfiguration('yaw_pose'),
            # Timeout (seconds) to wait for Gazebo to be ready
            '-timeout', '60.0'
        ],
        output='screen'
    )

    tf_base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.05', '0.0', '0.0', '0.0', 'base_footprint', 'base_link'],
    )

    # ========== Build Launch Description ==========
    return LaunchDescription([
        # Environment variable setup
        set_model_path,

        # Launch argument declarations
        declare_robot_name,
        declare_x_pose,
        declare_y_pose,
        declare_z_pose,
        declare_yaw_pose,
        declare_use_sim_time,
        tf_base_footprint_to_base_link,

        # Node execution
        spawn_entity_node,
    ])

