from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import UnlessCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_protobot_description = get_package_share_directory('protobot_description')
    pkg_protobot_controller = get_package_share_directory('protobot_controller')

    urdf_arg = DeclareLaunchArgument(
        'model', default_value='protobot.urdf.xacro',
        description='Name of the URDF file to load'
    )
    urdf_file_path = PathJoinSubstitution(
        [pkg_protobot_description, "urdf",
         LaunchConfiguration('model')]
    )
    rviz_config_path = PathJoinSubstitution(
        [pkg_protobot_description, "rviz",
         "display.rviz"]
    )
    controller_file_path = PathJoinSubstitution(
        [pkg_protobot_controller, "config",
         "controllers.yaml"]
    )

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        condition=UnlessCondition(LaunchConfiguration("is_sim")),
        parameters=[{'robot_description': Command(['xacro', ' ', urdf_file_path, " is_sim:=False"])}]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        condition=UnlessCondition(LaunchConfiguration("is_sim")),
        parameters=[
            {"robot_description": Command(['xacro', ' ', urdf_file_path]),
             "use_sim_time": LaunchConfiguration("is_sim")},
             controller_file_path
        ],
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=UnlessCondition(LaunchConfiguration("is_sim")),
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", 
            "--controller-manager", 
            "/controller_manager", 
        ]
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller", 
            "--controller-manager", 
            "/controller_manager", 
        ]
    )

    return LaunchDescription([
        urdf_arg,
        is_sim_arg,
        robot_state_publisher_node,
        controller_manager,
        rviz2_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ])