from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    declare_args = [
		DeclareLaunchArgument('anc0', default_value='0.0,0.0',
													 description='Anchor 1 location (x,y)'),
		DeclareLaunchArgument('anc1', default_value='-0.4375,0.3733615',
													 description='Anchor 2 location (x,y)'),
		DeclareLaunchArgument('anc2', default_value='0.4375,0.3733615',
													 description='Anchor 3 location (x,y)'),
		DeclareLaunchArgument('anc3', default_value='0,0',
													 description='Anchor 4 location (x,y)'),
		DeclareLaunchArgument('anc4', default_value='0,0',
													 description='Anchor 5 location (x,y)'),
				   ]    
    
    uwb_rcv_node = Node(
        package='uwb_test',
        executable='uwb_rcv',
        name='uwb_rcv',
        output='screen'
    )
    
    uwb_tf_node = Node(
        package='uwb_test',
        executable='uwb_tf',
        name='uwb_tf',
        parameters=[{
			'anc0': LaunchConfiguration('anc0'),
			'anc1': LaunchConfiguration('anc1'),
			'anc2': LaunchConfiguration('anc2'),
			'anc3': LaunchConfiguration('anc3'),
			'anc4': LaunchConfiguration('anc4'),
        }],
        output='screen'
    )
    
    # --- Return LaunchDescription ---
    return LaunchDescription(declare_args + [
        uwb_rcv_node,
        uwb_tf_node
    ])
