# File: velodyne_preprocess/launch/velodyne_preprocess.launch.py

import os
import ament_index_python.packages
import launch
import launch_ros.actions
import yaml

def generate_launch_description():
    share_dir = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    params_file = os.path.join(share_dir, 'config', 'VLP16-velodyne_convert_node-params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['velodyne_convert_node']['ros__parameters']
    params['calibration'] = os.path.join(share_dir, 'params', 'VLP16db.yaml')

    velodyne_convert_node = launch_ros.actions.Node(
        package='velodyne_pointcloud',
        executable='velodyne_convert_node',
        output='both',
        parameters=[params]
    )

    return launch.LaunchDescription([
        velodyne_convert_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=velodyne_convert_node,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown()
                )],
            )
        ),
    ])