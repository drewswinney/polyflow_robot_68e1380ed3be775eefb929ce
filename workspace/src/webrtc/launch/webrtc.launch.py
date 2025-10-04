# launch/webrtc.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_id",
            default_value="68e1380ed3be775eefb929ce",
            description="Unique ID of the robot"
        ),
        DeclareLaunchArgument(
            "signaling_url",
            default_value="{{SIGNALING_URL}}",
            description="WebSocket URL of the signaling server"
        ),
        DeclareLaunchArgument(
            "auth_token",
            default_value="",
            description="Optional auth token for signaling server"
        ),

        Node(
            package="polyflow_webrtc",
            executable="webrtc",         # From setup.py console_scripts
            name="webrtc_client",
            output="screen",
            parameters=[{
                "robot_id": LaunchConfiguration("robot_id"),
                "signaling_url": LaunchConfiguration("signaling_url"),
                "auth_token": LaunchConfiguration("auth_token"),
            }],
        ),
    ])
