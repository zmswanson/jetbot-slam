from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("jetbot_base")

    # Wheel odom params
    params_file_arg = DeclareLaunchArgument(
        "wheel_odom_params",
        default_value=os.path.join(pkg_share, "config", "wheel_odom.yaml"),
        description="Path to wheel odometry YAML (jetbot_wheel_odom).",
    )

    # RealSense params (passed to rs_launch.py)
    realsense_params_arg = DeclareLaunchArgument(
        "realsense_params",
        default_value=os.path.join(pkg_share, "config", "realsense_lowpower.yaml"),
        description="Path to RealSense YAML for rs_launch.py config_file.",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulated time.",
    )

    # URDF (xacro -> robot_description)
    urdf_path = os.path.join(pkg_share, "urdf", "jetbot.urdf.xacro")
    robot_description = ParameterValue(
        Command(["xacro ", urdf_path]),
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"robot_description": robot_description},
        ],
    )

    wheel_odom = Node(
        package="jetbot_base",
        executable="jetbot_wheel_odom",
        name="jetbot_wheel_odom",
        output="screen",
        parameters=[LaunchConfiguration("wheel_odom_params")],
    )

    # RealSense: include official launch file, passing our config YAML
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py")
        ]),
        launch_arguments={
            "config_file": LaunchConfiguration("realsense_params"),
        }.items(),
    )

    return LaunchDescription([
        params_file_arg,
        realsense_params_arg,
        use_sim_time_arg,
        robot_state_publisher,
        wheel_odom,
        realsense_launch,
    ])
