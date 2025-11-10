from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            get_package_share_directory("jetbot_base"),
            "config",
            "wheel_odom.yaml"
        ),
        description="Path to YAML file with wheel odometry parameters."
    )

    return LaunchDescription([
        params_file_arg,
        Node(
            package="jetbot_base",
            executable="jetbot_wheel_odom",
            name="jetbot_wheel_odom",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
