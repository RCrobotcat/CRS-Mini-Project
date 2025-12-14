#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ========== 路径配置 ==========
    # 获取当前功能包的launch目录路径
    launch_file_dir = os.path.join(
        get_package_share_directory('robot_simulation'),
        'launch'
    )

    # ========== Launch参数声明 ==========
    # --- 世界环境参数 ---
    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='2.world',
        description='Gazebo世界文件名称'
    )

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='是否启动Gazebo GUI'
    )

    declare_verbose = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='是否输出详细日志'
    )

    declare_pause = DeclareLaunchArgument(
        'pause',
        default_value='false',
        description='启动时是否暂停仿真'
    )

    # --- 机器人参数 ---
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='robotcar',
        description='机器人名称/命名空间'
    )

    declare_x_pose = DeclareLaunchArgument(
        'x_pose',
        default_value='5',
        description='机器人初始X坐标（米）'
    )

    declare_y_pose = DeclareLaunchArgument(
        'y_pose',
        default_value='-6',
        description='机器人初始Y坐标（米）'
    )

    declare_z_pose = DeclareLaunchArgument(
        'z_pose',
        default_value='0.05',
        description='机器人初始Z坐标（米）'
    )

    declare_yaw_pose = DeclareLaunchArgument(
        'yaw_pose',
        default_value='-3.14',
        description='机器人初始偏航角（弧度）'
    )

    # --- 通用参数 ---
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    )

    # ========== 子Launch文件包含 ==========
    # 1. 启动Gazebo世界环境
    # 负责启动gzserver和gzclient，加载世界文件
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

    # 2. 生成机器人模型
    # 负责发布机器人描述并在Gazebo中spawn机器人实体
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
        launch_arguments={'use_sim_time':  LaunchConfiguration('use_sim_time')}.items()
    )

    # ========== 构建Launch描述 ==========
    return LaunchDescription([
        # --- 参数声明 ---
        # 世界环境参数
        declare_world_name,
        declare_gui,
        declare_verbose,
        declare_pause,

        # 机器人参数
        declare_robot_name,
        declare_x_pose,
        declare_y_pose,
        declare_z_pose,
        declare_yaw_pose,

        # 通用参数
        declare_use_sim_time,

        # --- 启动顺序 ---
        # 注意：虽然这里按顺序写，但实际上launch系统会并行启动
        # robot_spawn中的spawn_entity有超时机制，会等待Gazebo准备就绪
        launch_empty_world,    # 先启动世界环境
        launch_robot_spawn,    # 然后生成机器人
        robot_state_publisher_cmd
    ])
