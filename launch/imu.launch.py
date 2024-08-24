from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='esp32_imu',
            executable='imu_publisher.py',
            name='imu_publisher',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='esp32_imu',
            executable='motion_estimator.py',
            name='motion_estimator',
            output='screen',
            emulate_tty=True,
        )
    ])
