"""
Baseline launch: runs tactical_racing_node with only the optimal raceline loaded.
Useful for verifying pure pursuit tracking before enabling tactical logic.
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('multiraceline_racing')
    config = os.path.join(pkg, 'config', 'params.yaml')

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
                    'optimal_csv': os.path.join(racelines_dir, 'optimal.csv'),
                    # Leaving other raceline paths empty causes graceful fallback to optimal
                }
            ],
            output='screen',
        )
    ])
