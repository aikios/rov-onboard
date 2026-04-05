from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url': 'serial:///dev/ttyACM0:115200',
            'gcs_url': '',
            'target_system_id': 1,
            'target_component_id': 1,
            'fcu_protocol': 'v2.0',
        }],
    )

    return LaunchDescription([
        mavros_node,
    ])
