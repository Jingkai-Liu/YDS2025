from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
         Node(
            package='velodyne_pointcloud',
            executable='cloud_node',
            name='velodyne_convert_node',
            parameters=[{
                # 'min_range': 0.4,
                # 'max_range': 130.0,
                # 'view_direction': 0.0,
                # 'view_width': 360.0,
                'model': 'VLP16',
            }]
            # remappings=[
            #     ('velodyne_packets', '/velodyne_packets'),
            #     ('velodyne_points', '/velodyne_points')
        #     # ]
        )
    ])
