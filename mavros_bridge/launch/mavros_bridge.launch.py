import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
)
from launch_ros.actions import (
    Node,
    ComposableNodeContainer,
)
from launch_ros.descriptions import ComposableNode

zed_package = get_package_share_directory('zed_ros2_minimal')
mavros_package = get_package_share_directory('mavros_bridge')
robot_viewer_package = get_package_share_directory('uosm_robot_viewer')
xacro_path = os.path.join(robot_viewer_package, 'urdf', 'dogpa.urdf.xacro')

zed_config_common = os.path.join(
    zed_package,
    'config',
    'common_stereo.yaml'
)
zed_config_camera = os.path.join(
    zed_package,
    'config',
    'zedm.yaml'
)

px4_config_path = os.path.join(
    get_package_share_directory('mavros_bridge'),
    'config', 'px4_config.yaml'
)
px4_pluginlists_path = os.path.join(
    get_package_share_directory('mavros_bridge'),
    'config', 'px4_pluginlists.yaml'
)

def generate_launch_description():
    # Declare arguments
    launch_foxglove_arg = DeclareLaunchArgument(
        'launch_foxglove',
        default_value='false',
        description='Whether to launch Foxglove Bridge',
        choices=['true', 'false']
    )

    # Create the composable node container
    container = ComposableNodeContainer(
        name='mavros_bridge_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # ZED Component
            ComposableNode(
                package='zed_ros2_minimal',
                plugin='stereolabs::ZedCamera',
                name='zed_node',
                parameters=[
                # YAML files
                  zed_config_common,  # Common parameters
                  zed_config_camera,  # Camera related parameters
                  {
                    'pos_tracking.publish_tf': False,
                    'pos_tracking.publish_map_tf': False,
                    'sensors.publish_imu_tf': True,
                    'debug_common': False,
                    'debug_video_depth': False,
                    'debug_camera_controls': False,
                    'debug_point_cloud': False,
                    'debug_positional_tracking': False,
                    'debug_sensors': False,
                    'debug_advanced': False,
                  }
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='mavros_bridge',
                plugin='uosm::mavros::MavrosBridgeComponent',
                name='mavros_bridge',
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Robot State Publisher Component
            ComposableNode(
                package='robot_state_publisher',
                plugin='robot_state_publisher::RobotStatePublisher',
                name='robot_state_publisher',
                parameters=[{
                    'robot_description': Command(['xacro ', xacro_path]),
                    'package_path': mavros_package
                }]
            ),
        ],
        output='screen',
    )

    # Mavros Node
    mavros = Node(
        package='mavros',
        executable='mavros_node',
        parameters=[
            px4_pluginlists_path,
            px4_config_path,
        {    
            # 'fcu_url': '/dev/ttyTHS1:1000000',
            'fcu_url': '/dev/ttyACM0:2000000',
            #'gcs_url': 'udp://@192.168.230.101',
            'gcs_url': 'udp://@10.42.0.189',
            #'gcs_url': 'udp://@10.100.18.240',
            'tgt_system': 1,
            'tgt_component': 1,
            'fcu_protocol': "v2.0",
            'respawn_mavros': "false",
            'namespace': "mavros",
        }],
    )

    # Joint State Publisher
    jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen"
    )

    return LaunchDescription([
        # Arguments
        launch_foxglove_arg,
        
        # Nodes and containers
        container,
        mavros,
        jsp,
    ])
