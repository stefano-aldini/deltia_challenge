from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('deltia_challenge')
    config_file = os.path.join(pkg_share, 'config', 'placements.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.11',
            description='IP address of the robot'
        ),
        DeclareLaunchArgument(
            'use_gripper',
            default_value='true',
            description='Whether to use the gripper'
        ),
        DeclareLaunchArgument(
            'use_camera',
            default_value='true',
            description='Whether to use the camera'
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Whether there is no hardware'
        ),

        # Launch Franka Bringup for the REAL robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('franka_bringup'),
                    'launch',
                    'franka.launch.py'
                ])
            ]),
            launch_arguments={
                'robot_ip': LaunchConfiguration('robot_ip'),
                'load_gripper': LaunchConfiguration('use_gripper'),
                'use_fake_hardware': 'false',
                'fake_sensor_commands': 'false',
                'launch_jsp': 'true',
            }.items(),
            condition=UnlessCondition(LaunchConfiguration('use_fake_hardware'))
        ),

        # Launch Franka Bringup for FAKE hardware (simulation)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('franka_bringup'),
                    'launch',
                    'franka.launch.py'
                ])
            ]),
            launch_arguments={
                'robot_ip': LaunchConfiguration('robot_ip'),
                'load_gripper': LaunchConfiguration('use_gripper'),
                'use_fake_hardware': 'true',
                'fake_sensor_commands': 'false',
                'launch_jsp': 'false',
            }.items(),
            condition=IfCondition(LaunchConfiguration('use_fake_hardware'))
        ),

        # Launch our own JSP for fake hardware mode
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'source_list': ['joint_states', 'franka_gripper/joint_states'],
                'rate': 30,
            }],
            condition=IfCondition(LaunchConfiguration('use_fake_hardware'))
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_robot_camera',
            arguments=['1', '1', '1', '0', '0', '1.570796', 'fr3_link0', 'camera_link']
        ),

        # Include the spatial detection launch file from depthai_ros_driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('depthai_ros_driver'),
                    'launch',
                    'spatial_detection.launch.py'
                ])
            ]),
            condition=IfCondition(LaunchConfiguration('use_camera')),
            launch_arguments={
                'name': 'camera',
                'parent_frame': 'camera_link',
                'params_file': PathJoinSubstitution([
                    FindPackageShare('deltia_challenge'),
                    'config',
                    'spatial.yaml'
                ])
            }.items()
        ),

        # Include the RViz launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('deltia_challenge'),
                    'launch',
                    'rviz.launch.py'
                ])
            ])
        ),

        # Launch the sorting node
        Node(
            package='deltia_challenge',
            executable='sorting_node',
            name='sorting_node',
            output='screen'
        ),

        # Launch the action node
        Node(
            package='deltia_challenge',
            executable='action_node',
            name='action_node',
            output='screen',
            parameters=[config_file]
        ),

        # Launch the GUI node
        Node(
            package='deltia_challenge',
            executable='gui_node.py',
            name='control_gui',
            output='screen'
        ),
    ])
