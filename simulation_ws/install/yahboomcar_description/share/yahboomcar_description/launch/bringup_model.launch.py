import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    package_name = 'yahboomcar_description'
    urdf_name = 'MicroROS.urdf'
    robot_name_in_model = 'yahboomcar'

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, "urdf", urdf_name)

    # Get the path of the gazebo_ros package
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=str(urdf_model_path),
        description='Absolute path to robot URDF file'
    )

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Set to "false" to run headless'
    )

    verbose_arg = DeclareLaunchArgument(
        name='verbose',
        default_value='true',
        description='Set to "true" for verbose output'
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Get the default world file path
    world_path = os.path.join(pkg_gazebo_ros, 'worlds', 'empty.world')

    # Start Gazebo server using the standard ROS 2 launch mechanism
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

    # Start Gazebo client (GUI) using the standard ROS 2 launch mechanism
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={
            'verbose': LaunchConfiguration('verbose')
        }.items()
    )

    # Gazebo spawn entity with extended timeout
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name_in_model,
            '-topic', 'robot_description',
            '-timeout', '60.0'  # Increase timeout duration
        ],
        output='screen'
    )

    # LaunchDescription
    ld = LaunchDescription()

    # Add argument declarations
    ld.add_action(model_arg)
    ld.add_action(gui_arg)
    ld.add_action(verbose_arg)

    # Start robot state publishers first
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)

    # Then start Gazebo (server first, then client)
    ld.add_action(gzserver_launch)
    ld.add_action(gzclient_launch)

    # Finally spawn the robot
    ld.add_action(spawn_entity_cmd)

    return ld

