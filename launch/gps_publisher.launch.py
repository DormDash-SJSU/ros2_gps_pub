from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_gps_pub',
            executable='gps_publisher',
            name='gps_publisher',
            parameters=[
                {'serial_port': '/dev/ttyUSB1'},
                {'baud_rate': 9600},
                {'publish_rate': 1.0}
            ],
            output='screen'
        )
    ])
