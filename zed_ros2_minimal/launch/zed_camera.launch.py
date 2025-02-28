# Copyright 2024 Stereolabs
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
    LogInfo
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    TextSubstitution
)
from launch_ros.actions import (
    Node,
    ComposableNodeContainer,
    LoadComposableNodes
)
from launch_ros.descriptions import ComposableNode

# ZED Configurations to be loaded by ZED Node
default_config_common = os.path.join(
    get_package_share_directory('zed_ros2_minimal'),
    'config',
    'common'
)

# URDF/xacro file to be loaded by the Robot State Publisher node
default_xacro_path = os.path.join(
    get_package_share_directory('zed_ros2_minimal'),
    'urdf',
    'zed_descr.urdf.xacro'
)


def launch_setup(context, *args, **kwargs):
    return_array = []

    wrapper_dir = get_package_share_directory('zed_ros2_minimal') 

    # Launch configuration variables
    container_name = LaunchConfiguration('container_name')
    namespace = LaunchConfiguration('namespace')
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')
    
    node_name = LaunchConfiguration('node_name')

    ros_params_override_path = LaunchConfiguration('ros_params_override_path')

    config_common_path = LaunchConfiguration('config_path')

    serial_number = LaunchConfiguration('serial_number')

    publish_urdf = LaunchConfiguration('publish_urdf')
    publish_tf = LaunchConfiguration('publish_tf')
    publish_map_tf = LaunchConfiguration('publish_map_tf')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf')
    xacro_path = LaunchConfiguration('xacro_path')

    custom_baseline = LaunchConfiguration('custom_baseline')

    container_name_val = container_name.perform(context)
    namespace_val = namespace.perform(context)
    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)
    node_name_val = node_name.perform(context)
    custom_baseline_val = custom_baseline.perform(context)

    if (camera_name_val == ''):
        camera_name_val = 'zedm'

    config_camera_path = os.path.join(
        get_package_share_directory('zed_ros2_minimal'),
        'config',
        camera_model_val + '.yaml'
    )

    if (camera_model_val == 'zedm' or 
        camera_model_val == 'zed2i' or 
        camera_model_val == 'virtual'):
        config_common_path_val = default_config_common + '_stereo.yaml'

    info = 'Using common configuration file: ' + config_common_path_val
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    config_camera_path = os.path.join(
        get_package_share_directory('zed_ros2_minimal'),
        'config',
        camera_model_val + '.yaml'
    )
    
    info = 'Using camera configuration file: ' + config_camera_path
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))
    
    ros_params_override_path_val = ros_params_override_path.perform(context)
    if(ros_params_override_path_val != ''):        
        info = 'Using ROS parameters override file: ' + ros_params_override_path_val
        return_array.append(LogInfo(msg=TextSubstitution(text=info)))

    # Xacro command with options
    xacro_command = []
    xacro_command.append('xacro')
    xacro_command.append(' ')
    xacro_command.append(xacro_path.perform(context))
    xacro_command.append(' ')
    xacro_command.append('camera_name:=')
    xacro_command.append(camera_name_val)
    xacro_command.append(' ')
    xacro_command.append('camera_model:=')
    xacro_command.append(camera_model_val)
    xacro_command.append(' ')
    xacro_command.append('custom_baseline:=')
    xacro_command.append(custom_baseline_val)   

    # Robot State Publisher node
    rsp_name = camera_name_val + '_state_publisher'
    rsp_node = Node(
        condition=IfCondition(publish_urdf),
        package='robot_state_publisher',
        namespace=namespace_val,
        executable='robot_state_publisher',
        name=rsp_name,
        output='screen',
        parameters=[{
            'robot_description': Command(xacro_command)
        }]
    )
    return_array.append(rsp_node)

    # ROS 2 Component Container
    if(container_name_val == ''):
        container_name_val='zed_container'
        container_exec='component_container_isolated'
        
        zed_container = ComposableNodeContainer(
                name=container_name_val,
                namespace=namespace_val,
                package='rclcpp_components',
                executable=container_exec,
                arguments=['--use_multi_threaded_executor','--ros-args', '--log-level', 'info'],
                output='screen',
        )
        return_array.append(zed_container)

    # ZED Node parameters
    node_parameters = [
            # YAML files
            config_common_path_val,  # Common parameters
            config_camera_path,  # Camera related parameters
    ]
    
    if( ros_params_override_path_val != ''):
        node_parameters.append(ros_params_override_path)

    node_parameters.append( 
            # Launch arguments must override the YAML files values
            {
                'general.camera_name': camera_name_val,
                'general.camera_model': camera_model_val,
                'general.serial_number': serial_number,
                'pos_tracking.publish_tf': publish_tf,
                'pos_tracking.publish_map_tf': publish_map_tf,
                'sensors.publish_imu_tf': publish_imu_tf,
            }
    )

    # ZED Wrapper component
    if( camera_model_val=='zedm' or
        camera_model_val=='zed2i' or
        camera_model_val=='virtual'):
        zed_component = ComposableNode(
            package='zed_ros2_minimal',
            namespace=namespace_val,
            plugin='stereolabs::ZedCamera',
            name=node_name_val,
            parameters=node_parameters,
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    full_container_name = container_name_val
    info = 'Loading ZED node `' + node_name_val + '` in container `' + full_container_name + '`'
    return_array.append(LogInfo(msg=TextSubstitution(text=info)))
    
    load_composable_node = LoadComposableNodes(
        target_container=full_container_name,
        composable_node_descriptions=[zed_component]
    )
    return_array.append(load_composable_node)

    return return_array

def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
            DeclareLaunchArgument(
                'camera_name',
                default_value=TextSubstitution(text='zed'),
                description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.'),
            DeclareLaunchArgument(
                'camera_model',
                description='[REQUIRED] The model of the camera. Using a wrong camera model can disable camera features.',
                choices=['zedm', 'zed2i', 'virtual']),
            DeclareLaunchArgument(
                'container_name',
                default_value='',
                description='The name of the container to be used to load the ZED component. If empty (default) a new container will be created.'),
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='The namespace of the node. If empty (default) the camera name is used.'),
            DeclareLaunchArgument(
                'node_name',
                default_value='zed_node',
                description='The name of the zed_ros2_minimal node. All the topic will have the same prefix: `/<camera_name>/<node_name>/`. If a namespace is specified, the node name is replaced by the camera name.'),
            DeclareLaunchArgument(
                'ros_params_override_path',
                default_value='',
                description='The path to an additional parameters file to override the default values.'),
            DeclareLaunchArgument(
                'serial_number',
                default_value='0',
                description='The serial number of the camera to be opened. It is mandatory to use this parameter in multi-camera rigs to distinguish between different cameras.'),
            DeclareLaunchArgument(
                'publish_urdf',
                default_value='true',
                description='Enable URDF processing and starts Robot State Published to propagate static TF.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'publish_tf',
                default_value='false',
                description='Enable publication of the `odom -> camera_link` TF.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'publish_map_tf',
                default_value='false',
                description='Enable publication of the `map -> odom` TF. Note: Ignored if `publish_tf` is False.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'publish_imu_tf',
                default_value='true',
                description='Enable publication of the IMU TF. Note: Ignored if `publish_tf` is False.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'xacro_path',
                default_value=TextSubstitution(text=default_xacro_path),
                description='Path to the camera URDF file as a xacro file.'),            
            DeclareLaunchArgument(
                'custom_baseline',
                default_value='0.0',
                description='Distance between the center of ZED X One cameras in a custom stereo rig.'),
            OpaqueFunction(function=launch_setup)
        ]
    )
