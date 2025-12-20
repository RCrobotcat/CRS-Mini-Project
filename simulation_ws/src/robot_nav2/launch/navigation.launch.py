import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Locate the 'launch' directory of the Nav2 (ROS 2 Navigation Stack) package 'nav2_bringup'
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
        
    # Locate the shared directory of our own 'robot_nav2' package
    robot_nav2_dir = get_package_share_directory('robot_nav2')

    # RViz configuration file path
    rviz_config_dir = os.path.join(
        get_package_share_directory('robot_nav2'),
        'rviz',
        'navigation.rviz')

    # Build the default absolute path to the map file
    default_map_path = os.path.join(robot_nav2_dir, 'map', 'map.yaml')
    
    # Build the default absolute path to the Nav2 parameter file
    default_param_path = os.path.join(robot_nav2_dir, 'param', 'nav2_params.yaml')

    # Declare the 'use_sim_time' argument to indicate whether nodes should use
    # simulation time (e.g., /clock topic published by Gazebo)
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',  # Use simulation clock by default
        description='Whether to use simulation (Gazebo) clock')

    # Declare the 'map' argument to specify the full path to the map file (.yaml)
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,  # Default to the path defined above
        description='Full path to map file to load')

    # Declare the 'params_file' argument to specify the Nav2 parameter file (.yaml)
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_param_path,  # Default to the path defined above
        description='Full path to param file to load')

    # Include the official Nav2 launch file 'bringup_launch.py'
    include_nav2_bringup = IncludeLaunchDescription(
        # Specify the source of the launch file to include
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        # Pass the declared arguments to the Nav2 bringup launch file
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file')
        }.items()  # .items() is required to convert to iterable key-value pairs
    )

    # RViz node for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map)
    ld.add_action(declare_params_file)
    ld.add_action(include_nav2_bringup)
    ld.add_action(rviz_node)

    return ld

