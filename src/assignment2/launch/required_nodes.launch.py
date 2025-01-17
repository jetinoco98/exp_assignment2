import launch
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        LogInfo(
            msg="Launching all required nodes..."
        ),
        Node(
            package='assignment2',
            executable='inspect',
            name='inspect',
            output='screen'
        ),
        Node(
            package='assignment2',
            executable='move',
            name='move',
            output='screen'
        ),
        Node(
            package='assignment2',
            executable='waypoint_manager',
            name='waypoint_manager',
            output='screen'
        ),
        Node(
            package='ros2_aruco',
            executable='aruco_node',
            name='aruco_node',
            output='screen'
        )
    ])

