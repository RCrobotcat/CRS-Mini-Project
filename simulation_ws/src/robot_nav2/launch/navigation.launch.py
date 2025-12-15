import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():


    # 找到 Nav2 (ROS 2 导航栈) 的 'nav2_bringup' 包的 'launch' 目录
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
        
    # 找到我们自己的 'robot_nav2' 功能包的共享目录
    robot_nav2_dir = get_package_share_directory('robot_nav2')

    rviz_config_dir = os.path.join(
        get_package_share_directory('robot_nav2'),
        'rviz',
        'navigation.rviz')
    # 构建地图文件的默认绝对路径
    default_map_path = os.path.join(robot_nav2_dir, 'map', 'map.yaml')
    
    # 构建 Nav2 参数文件的默认绝对路径
    default_param_path = os.path.join(robot_nav2_dir, 'param', 'nav2_params.yaml')

    # 声明 'use_sim_time' 参数,告知所有节点是否使用仿真时钟 (例如 Gazebo 发布的 /clock 话题)
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',  # 默认使用仿真时钟
        description='Whether to use simulation (Gazebo) clock')

    # 声明 'map' 参数,指定要加载的地图文件 (.yaml) 的完整路径
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map_path, # 默认值为我们上面找到的路径
        description='Full path to map file to load')

    # 声明 'params_file' 参数,指定 Nav2 使用的参数配置文件 (.yaml)
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_param_path, # 默认值为我们上面找到的路径
        description='Full path to param file to load')


    # include调用 Nav2 官方提供的 'bringup_launch.py'，
    include_nav2_bringup = IncludeLaunchDescription(
        # 指定要包含的启动文件的来源
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        # 传递前面声明参数的值。
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file')
        }.items() # .items() 是必须的，用于将其转换为可迭代的键值对
    )


    rviz_node= Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen')
    

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map)
    ld.add_action(declare_params_file)
    ld.add_action(include_nav2_bringup)
    ld.add_action(rviz_node)


    return ld
