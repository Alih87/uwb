from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swri_transform_util',
            executable='initialize_origin.py',
            name='initialize_origin',
            output='screen',
            parameters=[
                {'local_xy_frame': 'map'},
                {'local_xy_origin': 'scout'},
                {'local_xy_origins': """[
                    {
                        'name': 'scout',
                        'latitude': 35.8376318,
                        'longitude': 127.1300570,
                        'altitude': 0.0,
                        'heading': 0.0
                    }
                ]"""}
            ]
        ),
        
        Node(
            package='mapviz',
            executable='mapviz',
            name='mapviz',
            output='screen'
        )
    ])
