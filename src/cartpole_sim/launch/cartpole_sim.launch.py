from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_installation_path = FindPackageShare("cartpole_sim")
    urdf_xacro_file_path = PathJoinSubstitution(
        [pkg_installation_path, "urdf", "cartpole.urdf.xacro"]
    )
    rviz_config_path = PathJoinSubstitution(
        [pkg_installation_path, "rviz", "default.rviz"]
    )

    robot_description_content = Command(["xacro ", urdf_xacro_file_path])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_node",
        arguments=["-d", rviz_config_path],
    )

    cartpole_physics_node = Node(
        package="cartpole_sim",
        executable="cartpole_physics_node",
        name="cartpole_physics_node",
    )

    return LaunchDescription(
        [robot_state_publisher_node, rviz_node, cartpole_physics_node]
    )
