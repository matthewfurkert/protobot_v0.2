from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    pkg_protobot_description = get_package_share_directory('protobot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(pkg_protobot_description).parent.resolve())]
    )

    urdf_arg = DeclareLaunchArgument(
        name='model',
        default_value='protobot.urdf.xacro',
        description='Name of the URDF file to load'
    )
    gz_config_arg = DeclareLaunchArgument(
        name='config',
        default_value='basic.config',
        description='Name of the gz client configuration file to load'
    )
    gz_world_arg = DeclareLaunchArgument(
        name='world', 
        default_value='empty.sdf',
        description='Name of the Gazebo world file to load'
    )
    time_arg = DeclareLaunchArgument(
        name='use_sim_time', 
        default_value='true',
        description='Setting time based of simulation'
    )

    urdf_path = PathJoinSubstitution([pkg_protobot_description, "urdf", LaunchConfiguration('model')])
    gz_config_path = PathJoinSubstitution([pkg_protobot_description, "config", LaunchConfiguration('config')])
    gz_world_path = PathJoinSubstitution([pkg_protobot_description, "config", LaunchConfiguration('world')])
    gz_bridge_params_path = PathJoinSubstitution([pkg_protobot_description, "config", 'gz_bridge.yaml'])
    time = LaunchConfiguration('use_sim_time')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro', ' ', urdf_path]),
                     'use_sim_time': time}],
    )
    gz_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={'gz_args': [gz_world_path,
            TextSubstitution(text=' --gui-config '), gz_config_path,
            TextSubstitution(text=' -r -v -v1 --physics-engine gz-physics-bullet-featherstone-plugin')],
            'on_exit_shutdown': 'true'}.items()
    )
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "my_robot",
            "-topic", "robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.0", "-Y", "0.0"  # Initial spawn position
        ],
        parameters=[
            {'use_sim_time': time},
        ]
    )
    # Node to bridge messages like /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[
            {'config_file': gz_bridge_params_path},
            {'use_sim_time': time},
        ]
    )

    return LaunchDescription([
        gz_resource_path,
        urdf_arg,
        gz_config_arg,
        gz_world_arg,
        time_arg,
        robot_state_publisher_node,
        gz_launch,
        gz_spawn_entity,
        gz_bridge_node,
    ])