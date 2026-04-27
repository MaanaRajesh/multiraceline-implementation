import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('multiraceline_racing')
    config = os.path.join(pkg, 'config', 'params.yaml')

    # Point these at your actual waypoint CSVs.
    # The easiest workflow: generate them with raceline_UI, export to this package's
    # racelines/ directory, then set the paths here.
    racelines_dir = os.path.join(pkg, '..', '..', '..', 'racelines', 'levine')

    return LaunchDescription([
        Node(
            package='multiraceline_racing',
            executable='tactical_racing_node',
            name='tactical_racing_node',
            parameters=[
                config,
                {
                    'use_sim': True,
                    'centerline_csv':     os.path.join(racelines_dir, 'centerline.csv'),
                    'optimal_csv':        os.path.join(racelines_dir, 'optimal.csv'),
                    'inside_attack_csv':  os.path.join(racelines_dir, 'inside_attack.csv'),
                    'outside_attack_csv': os.path.join(racelines_dir, 'outside_attack.csv'),
                    'defensive_csv':      os.path.join(racelines_dir, 'defensive.csv'),
                }
            ],
            output='screen',
        )
    ])
