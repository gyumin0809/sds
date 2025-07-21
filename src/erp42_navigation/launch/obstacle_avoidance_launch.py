# erp42_navigation/launch/obstacle_avoidance_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='erp42_navigation',
            # 'executable'은 setup.py에 정의된 이름과 일치해야 합니다.
            executable='lidar_node',
            # 'name'은 ROS2 그래프에 표시될 노드의 이름입니다. 자유롭게 지정할 수 있습니다.
            name='lidar_sensor_node',
            output='screen'
        ),
        Node(
            package='erp42_navigation',
            executable='obstacle_avoider',
            name='obstacle_avoidance_node',
            output='screen'
        ),
        Node(
            package='erp42_navigation',
            executable='vehicle_driver',
            name='vehicle_driver_node',
            output='screen',
            parameters=[{'serial.port': 'COM4'}]  # 실제 포트 이름으로 변경하세요 (예: '/dev/ttyUSB0')
        ),
    ])
