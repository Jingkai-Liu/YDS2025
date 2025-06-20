from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            name='velodyne_driver',
            output='screen',
            parameters=[{
                'device_ip': '192.168.1.201',  # ← 这里改为你实际的雷达IP
                'frame_id': 'velodyne',
                'model': 'VLP16',
                'rpm': 600.0,
                'port': 2368
            }]
        ),
        # Node(
        #     package='velodyne_pointcloud',
        #     executable='cloud_node',
        #     name='velodyne_convert_node',
        #     parameters=[{
        #         # 'min_range': 0.4,
        #         # 'max_range': 130.0,
        #         # 'view_direction': 0.0,
        #         # 'view_width': 360.0,
        #         'model': 'VLP16',
        #     }]
        #     # remappings=[
        #     #     ('velodyne_packets', '/velodyne_packets'),
        #     #     ('velodyne_points', '/velodyne_points')
        # #     # ]
        # ),
        Node(
            package='velodyne_distance',
            executable='distance_node',
            name='distance_node',
            output='screen',
        )
    ])
