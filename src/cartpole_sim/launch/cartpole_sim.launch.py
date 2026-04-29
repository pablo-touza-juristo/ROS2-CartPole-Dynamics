import launch
from launch import LaunchDescription
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.actions.lifecycle_node import lifecycle_msgs
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_installation_path = FindPackageShare("cartpole_sim")
    urdf_xacro_file_path = PathJoinSubstitution(
        [pkg_installation_path, "urdf", "cartpole.urdf.xacro"]
    )
    rviz_config_path = PathJoinSubstitution(
        [pkg_installation_path, "rviz", "default.rviz"]
    )

    config_path = PathJoinSubstitution(
        [pkg_installation_path, "config", "parameters.yaml"]
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

    cartpole_physics_node = LifecycleNode(
        namespace="",
        package="cartpole_sim",
        executable="cartpole_physics_node",
        name="cartpole_physics_node",
        parameters=[config_path],
    )

    on_configure_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=cartpole_physics_node,
            on_start=[
                LogInfo(msg="Process initiated, recovering information"),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            cartpole_physics_node
                        ),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )
                ),
            ],
        )
    )

    on_activate_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=cartpole_physics_node,
            goal_state="inactive",
            entities=[
                LogInfo(msg="Activating the node"),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            cartpole_physics_node
                        ),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            rviz_node,
            cartpole_physics_node,
            on_configure_event_handler,
            on_activate_event_handler,
        ]
    )
