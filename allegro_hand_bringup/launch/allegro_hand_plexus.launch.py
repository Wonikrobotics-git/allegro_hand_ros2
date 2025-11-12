import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    device = LaunchConfiguration('device', default='plexus')
    hand_side = LaunchConfiguration('hand', default='right') 
    ros2_control_hardware_type = LaunchConfiguration("ros2_control_hardware_type")
 
    # Robot Description
    robot_description_str = Command(
        ['xacro ', 
         PathJoinSubstitution([
            FindPackageShare('allegro_hand_bringup'),
            'config', device, 'single_hand', 'allegro_hand.urdf.xacro'
         ]),
         ' hand:=', hand_side, 
         ' ros2_control_hardware_type:=', ros2_control_hardware_type, 
        ]
    )
    robot_description = {'robot_description': robot_description_str} 

    # Path to the base controller configurations
    ros2_controllers_path = PathJoinSubstitution([
        FindPackageShare('allegro_hand_bringup'),
        'config', device, 'single_hand', 'ros2_controllers.yaml'
    ])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            ros2_controllers_path, 
        ],
        output='screen',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    ) 
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    allegro_hand_imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["allegro_hand_imu_broadcaster", "--controller-manager", "/controller_manager"],
    )

    allegro_hand_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["allegro_hand_position_controller", "-c", "/controller_manager"],
    ) 

    allegro_hand_position_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["allegro_hand_position_effort_controller", "-c", "/controller_manager", "--inactive"],
    ) 

    bringup_pkg_path = get_package_share_directory('allegro_hand_bringup')
    analyzer_config_path = os.path.join(
        bringup_pkg_path, 'config', 'plexus', 'single_hand', 'diagnostic_analyzers.yaml'
    )

    with open(analyzer_config_path, 'r') as f:
        analyzer_params = yaml.safe_load(f)
        
    diagnostic_aggregator_node = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        output='screen',
        # 'config_file' 대신, 파일 내용을 담은 딕셔너리를 직접 전달합니다.
        parameters=[analyzer_params]
    )


    rviz_file = PathJoinSubstitution([
        FindPackageShare('allegro_hand_bringup'), 'rviz', device, 'hand.rviz'
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'device',
            default_value='plexus',
            description='Allegro Hand device version. -- possible values: [v4, plexus]'),
        DeclareLaunchArgument(
            'hand',
            default_value='right',
            description='Specify which hand to use: right or left'),
        DeclareLaunchArgument(
            "ros2_control_hardware_type",
            default_value="physical_device",
            description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, physical_device, gazebo, isaac]"),

        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        allegro_hand_imu_broadcaster_spawner, 
        allegro_hand_position_controller_spawner, 
        allegro_hand_position_effort_controller,
        diagnostic_aggregator_node,
        rviz_node,
    ])