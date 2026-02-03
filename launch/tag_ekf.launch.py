import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    declare_args = [
        DeclareLaunchArgument('anc0', default_value='anc0', description='Anchor 1 location (x,y)'),
		DeclareLaunchArgument('anc1', default_value='anc1', description='Anchor 2 location (x,y)'),
		DeclareLaunchArgument('anc2', default_value='anc2', description='Anchor 3 location (x,y)'),
		DeclareLaunchArgument('anc3', default_value='anc3', description='Anchor 4 location (x,y)'),
		DeclareLaunchArgument('anc4', default_value='anc4', description='Anchor 5 location (x,y)'),
		DeclareLaunchArgument('tag_frame', default_value='tag_frame', description='Tag frame'),
		DeclareLaunchArgument('aux_frame', default_value='aux_frame', description='Intermediate global frame'),
		DeclareLaunchArgument('ekf_params', default_value='ekf_params', description='Intermediate global frame'),
    ]
    
    uwb_tf_node = Node(
        package='uwb_test',
        executable='uwb_tf',
        name=PythonExpression(['"uwb_tf_" + "', LaunchConfiguration('tag_frame'), '"']),
        parameters=[{
			'anc0': LaunchConfiguration('anc0'),
			'anc1': LaunchConfiguration('anc1'),
			'anc2': LaunchConfiguration('anc2'),
			'anc3': LaunchConfiguration('anc3'),
			'anc4': LaunchConfiguration('anc4'),
			'tag_frame': LaunchConfiguration('tag_frame'),
			'aux_frame': LaunchConfiguration('aux_frame')
        }],
        output='screen'
    )
    
    ekf_tf_node = Node(
        package='uwb_test',
        executable='ekf_tf',
        name=PythonExpression(['"ekf_tf_" + "', LaunchConfiguration('tag_frame'), '"']),
        parameters=[{
        'tag_frame': LaunchConfiguration('tag_frame')
        }],
        output='screen'
    )
    
    ekf_filter_node_fused = Node(
		package="robot_localization",
		executable="ekf_node",
		name=PythonExpression(['"ekf_filter_node_fused_" + "', LaunchConfiguration('tag_frame'), '"']),
		parameters=[{PythonExpression(['"ekf_filter_node_fused_" + "', LaunchConfiguration('tag_frame'), '"']): ""}, LaunchConfiguration('ekf_params')],
		remappings=[('odometry/filtered', PythonExpression(['"uwb/" + "', LaunchConfiguration('tag_frame'), '" + "/dyn_fused"']))]
	)
	
    ekf_filter_node_map = Node(
		package="robot_localization",
		executable="ekf_node",
		name=PythonExpression(['"ekf_filter_node_map_" + "', LaunchConfiguration('tag_frame'), '"']),
		parameters=[{PythonExpression(['"ekf_filter_node_map_" + "', LaunchConfiguration('tag_frame'), '"']): ""}, LaunchConfiguration('ekf_params')],
		remappings=[('odometry/filtered', PythonExpression(['"uwb/" + "', LaunchConfiguration('tag_frame'), '" + "/static_filtered"']))]
	)
	
	## Convert the filtered tag position to global GNSS position in UTM coordinates
    #navsat_tf_node = Node(
	#	package='robot_localization',
	#	executable='navsat_transform_node',
	#	name=PythonExpression(['"navsat_transform_node_" + "', LaunchConfiguration('tag_frame'), '"']),
	#	output='screen',
	#	parameters=[LaunchConfiguration('ekf_params')],
	#	remappings=[
	#		('gps/fix', '/ublox_gps_node/fix'),		# this needs to be tag location not base_link location
	#		('odometry/filtered', PythonExpression(['"uwb/" + "', LaunchConfiguration('tag_frame'), '" + "/static_filtered"']))
	#	]
	#)
    
    return LaunchDescription(declare_args + [
		uwb_tf_node,
		ekf_tf_node,
		ekf_filter_node_fused,
		ekf_filter_node_map
    ])
