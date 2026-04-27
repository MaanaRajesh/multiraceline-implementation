from launch import LaunchDescription                                                            
from launch_ros.actions import Node                                                                                                                                                             
def generate_launch_description():                                                                  
    racelines = '/home/maana/roboracer_ws/src/multi_raceline_imp/racelines/levine'
    config = '/home/maana/roboracer_ws/src/multi_raceline_imp/multiraceline_racing/multiraceline_racing/src/config/params.yaml'

    return LaunchDescription([
        Node(
            package='multiraceline_racing',
            executable='tactical_racing_node',
            name='tactical_racing_node',
            parameters=[
                config,
                {
                    'use_sim': True,
                    'optimal_csv':        racelines + '/optimal.csv',
                    'centerline_csv':     racelines + '/centerline.csv',
                    'inside_attack_csv':  racelines + '/inside_attack.csv',
                    'outside_attack_csv': racelines + '/outside_attack.csv',
                    'defensive_csv':      racelines + '/defensive.csv',
                }
            ],
            output='screen',
        )
    ])

