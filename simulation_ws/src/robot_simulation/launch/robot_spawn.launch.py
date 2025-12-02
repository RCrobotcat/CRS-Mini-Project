#!/usr/bin/env python3
"""
机器人生成Launch文件

功能说明：
    此launch文件负责在Gazebo仿真环境中生成机器人模型，包括：
    1. 发布机器人的URDF描述（通过robot_state_publisher）
    2. 在Gazebo中spawn机器人实体
    3. 设置机器人的初始位姿（位置和姿态）

主要节点：
    - robot_state_publisher: 发布机器人的TF变换和robot_description
    - spawn_entity.py: Gazebo的实体生成节点

Launch参数：
    - robot_name: 机器人名称/前缀 (默认: 'nanocar')
    - x_pose: 机器人初始X坐标 (默认: 0.0)
    - y_pose: 机器人初始Y坐标 (默认: 0.0)
    - z_pose: 机器人初始Z坐标 (默认: 0.0)
    - yaw_pose: 机器人初始偏航角/朝向 (默认: 0.0)
    - use_sim_time: 是否使用仿真时间 (默认: true)

使用示例：
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
    # ========== 路径配置 ==========
    # 获取功能包的路径
    pkg_share = FindPackageShare(package='robot_simulation').find('robot_simulation')

    # SDF模型文件的完整路径
    sdf_file = os.path.join(
        pkg_share,
        'models/turtlebot3_waffle/model.sdf',
    )

    # ========== 环境变量设置 ==========
    # 设置Gazebo模型路径，使Gazebo能够找到机器人的mesh文件和其他资源
    models_dir = os.path.join(pkg_share, 'models')
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=models_dir
    )

    # ========== Launch参数声明 ==========
    # 声明robot_name参数：机器人的名称，用于在多机器人仿真中区分不同机器人
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='micu_car',
        description='机器人名称，用于命名空间和实体标识'
    )

    # 声明初始位置参数：机器人在Gazebo世界中的初始X坐标
    declare_x_pose = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='机器人初始位置的X坐标（米）'
    )

    # 声明初始位置参数：机器人在Gazebo世界中的初始Y坐标
    declare_y_pose = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='机器人初始位置的Y坐标（米）'
    )

    # 声明初始位置参数：机器人在Gazebo世界中的初始Z坐标（高度）
    declare_z_pose = DeclareLaunchArgument(
        'z_pose',
        default_value='0.0',
        description='机器人初始位置的Z坐标（米）'
    )

    # 声明初始姿态参数：机器人的初始偏航角（绕Z轴旋转）
    declare_yaw_pose = DeclareLaunchArgument(
        'yaw_pose',
        default_value='0.0',
        description='机器人初始偏航角（弧度），0为正东方向'
    )

    # 声明仿真时间参数：是否使用Gazebo的仿真时间
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间（true/false）'
    )

    # ========== 节点定义 ==========
    # spawn_entity节点：在Gazebo中生成机器人实体
    # 功能：从SDF文件读取模型描述，在指定位姿生成机器人
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            # 实体名称，用于在Gazebo中标识此机器人
            '-entity', LaunchConfiguration('robot_name'),
            # 从SDF文件读取机器人描述
            '-file', sdf_file,
            # 设置机器人的初始位置（X, Y, Z）
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', LaunchConfiguration('z_pose'),
            # 设置机器人的初始姿态（Yaw角度，绕Z轴）
            '-Y', LaunchConfiguration('yaw_pose'),
            # 超时时间（秒），等待Gazebo准备就绪
            '-timeout', '60.0'
        ],
        output='screen'
    )

    # ========== 构建Launch描述 ==========
    return LaunchDescription([
        # 环境变量设置
        set_model_path,

        # Launch参数声明
        declare_robot_name,
        declare_x_pose,
        declare_y_pose,
        declare_z_pose,
        declare_yaw_pose,
        declare_use_sim_time,

        # 节点启动
        spawn_entity_node,
    ])
