"""
@file jetbot_motor_driver.launch.py
@brief Launch the Waveshare JetBot motor driver node.
@ingroup jetbot_base
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="jetbot_base",
            executable="jetbot_motor_driver_node",
            name="jetbot_motor_driver",
            output="screen",
            parameters=[{
                # Your verified settings:
                "i2c_device": "/dev/i2c-1",
                "i2c_addr": 0x60,

                # Start conservative; tune later
                "wheel_separation": 0.10,
                "max_linear_mps": 0.40,
                "max_angular_rps": 3.00,

                # Safety
                "cmd_timeout_sec": 1.00,

                # Wiring/convention toggles
                "left_inverted": False,
                "right_inverted": False,

                # Slew limits
                "max_accel_mps2": 1.00,
                "max_alpha_rps2": 6.00,
            }],
        )
    ])
