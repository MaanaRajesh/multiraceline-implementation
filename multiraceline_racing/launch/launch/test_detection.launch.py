"""
Test launch: runs tactical_racing_node with detection logging cranked up.
Use this to verify the opponent detector is picking up the second car before
enabling the state machine in a real race.
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
                    # Tighter detection params for testing — tune these
                    'detection_max_range':   4.0,
                    'detection_min_points':  2,
                    'detection_max_width':   0.5,
                    # Freeze state machine in SOLO so car doesn't tactically react
                    'follow_threshold_s':    0.0,
                    'defend_threshold_s':    0.0,
                }
            ],
            remappings=[
                ('/tactical_markers', '/test_detection_markers'),
            ],
            output='screen',
        )
    ])
