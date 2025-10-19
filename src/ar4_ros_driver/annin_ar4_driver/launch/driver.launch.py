from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    serial_port = LaunchConfiguration("serial_port")
    calibrate = LaunchConfiguration("calibrate")
    include_gripper = LaunchConfiguration("include_gripper")
    arduino_serial_port = LaunchConfiguration("arduino_serial_port")
    ar_model_config = LaunchConfiguration("ar_model")
    tf_prefix = LaunchConfiguration("tf_prefix")

    # Robot description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("annin_ar4_driver"), "urdf", "ar.urdf.xacro"]
            ),
            " ",
            "ar_model:=",
            ar_model_config,
            " serial_port:=",
            serial_port,
            " calibrate:=",
            calibrate,
            " tf_prefix:=",
            tf_prefix,
            " include_gripper:=",
            include_gripper,
            " arduino_serial_port:=",
            arduino_serial_port,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Controller config
    joint_controllers_cfg = PathJoinSubstitution(
        [FindPackageShare("annin_ar4_driver"), "config", "controllers.yaml"]
    )

    # Nodes
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ParameterFile(joint_controllers_cfg, allow_substs=True),
            {"tf_prefix": tf_prefix},
        ],
        remappings=[("~/robot_description", "robot_description")],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
    )

    spawn_joint_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
    )

    spawn_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        condition=IfCondition(include_gripper),
        output="screen",
    )

    # LaunchDescription
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"))
    ld.add_action(
        DeclareLaunchArgument(
            "calibrate", default_value="True", choices=["True", "False"]
        )
    )
    ld.add_action(DeclareLaunchArgument("tf_prefix", default_value=""))
    ld.add_action(
        DeclareLaunchArgument(
            "include_gripper", default_value="True", choices=["True", "False"]
        )
    )
    ld.add_action(
        DeclareLaunchArgument("arduino_serial_port", default_value="/dev/ttyUSB0")
    )
    ld.add_action(
        DeclareLaunchArgument(
            "ar_model", default_value="mk3", choices=["mk1", "mk2", "mk3"]
        )
    )

    # Core nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(controller_manager_node)

    # Event-based controller spawners
    ld.add_action(
        RegisterEventHandler(
            OnProcessStart(
                target_action=controller_manager_node,
                on_start=[
                    spawn_joint_state_broadcaster,
                    spawn_joint_controller,
                    spawn_gripper_controller,
                ],
            )
        )
    )

    return ld
