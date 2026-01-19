import os
import tempfile
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    DeclareLaunchArgument,
    AppendEnvironmentVariable,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    device = LaunchConfiguration("device").perform(context) 
    description_pkg_name = f"allegro_hand_{device}_description"
    description_pkg = get_package_share_directory(description_pkg_name)
    description_pkg_base = os.path.dirname(description_pkg)

    return [
        AppendEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH", value=description_pkg_base
        )
    ]


def create_controller_nodes(device, robot_controllers, joint_state_broadcaster, suffix=""):

    allegro_hand_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            f"allegro_hand_position_controller{suffix}",
            "--param-file",
            robot_controllers,
        ],
        output="screen",
    )

    allegro_hand_controller_gp_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            f"allegro_hand_posture_controller{suffix}",
            "--param-file",
            robot_controllers,
            "--inactive",
        ],
        output="screen",
    )

    allegro_hand_controller_ge_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            f"allegro_hand_grasp_controller{suffix}",
            "--param-file",
            robot_controllers,
            "--inactive",
        ],
        output="screen",
    )

    delayed_position_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[allegro_hand_position_controller_spawner],
        )
    )

    delayed_posture_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[allegro_hand_controller_gp_spawner],
        ),
        condition=IfCondition(PythonExpression(["'", device, "' == 'v4'"])),
    )

    delayed_grasp_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[allegro_hand_controller_ge_spawner],
        ),
        condition=IfCondition(PythonExpression(["'", device, "' == 'v4'"])),
    )

    return [
        delayed_position_controller, 
        delayed_posture_controller,
        delayed_grasp_controller,
    ]
 

def generate_launch_description():
    # 1. Launch Configurations
    use_sim_time = LaunchConfiguration("use_sim_time", default="true") 
    device = LaunchConfiguration("device", default="v4") # v4, plexus 
    hand_side = LaunchConfiguration("hand", default="right") # left, right  
    configuration = LaunchConfiguration("configuration", default="single_hand") # single_hand, dual_hand 

    # 2. Path Definitions
    ros2_control_pkg_share = get_package_share_directory("allegro_hand_bringup")

    # Xacro path
    xacro_file = PathJoinSubstitution(
        [
            ros2_control_pkg_share,
            "config",
            device,
            configuration,
            "allegro_hand.urdf.xacro",
        ]
    )

    robot_controllers = PathJoinSubstitution(
        [
            ros2_control_pkg_share,
            "config",
            device,
            configuration,
            "ros2_controllers.yaml",
        ]
    )

    # Gazebo GUI config path
    gazebo_gui_config_file = PathJoinSubstitution(
        [ros2_control_pkg_share, "visualize", device, "gazebo_gui.config"]
    )

    # 4. Process URDF
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
            " hand:=",
            hand_side,
            " use_sim:=true",
            " sim_gazebo_classic:=false",
            " ros2_control_hardware_type:=gazebo",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Nodes & Launch
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gz_args": ["-r -v 1 empty.sdf --gui-config ", gazebo_gui_config_file]
        }.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "allegro_hand",
            "-z",
            "0.5",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    delayed_joint_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster],
        )
    )

    def spawn_controllers(context, *args, **kwargs):
        config_str = context.perform_substitution(configuration)
        if config_str == "single_hand":
            return create_controller_nodes(
                device, robot_controllers, joint_state_broadcaster, suffix=""
            )
        else:
            return create_controller_nodes(
                device, robot_controllers, joint_state_broadcaster, suffix="_l"
            ) + create_controller_nodes(
                device, robot_controllers, joint_state_broadcaster, suffix="_r"
            )

    ld = LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("device", default_value="v4"),
            DeclareLaunchArgument("hand", default_value="right"),
            DeclareLaunchArgument("configuration", default_value="single_hand"),
            OpaqueFunction(function=launch_setup),
            gz_sim,
            bridge,
            robot_state_publisher,
            spawn_robot,
            delayed_joint_broadcaster, 
            OpaqueFunction(function=spawn_controllers),
        ]
    )

    return ld 

 
