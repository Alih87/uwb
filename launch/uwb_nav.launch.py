from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import TimerAction

STATIC_ANCHORS = {
"anc0":["0.0","-3.18"],
"anc3":["0.0","0.0"],
"anc4":["1.65","0.22"]
}

DYNAMIC_ANCHORS = {
"anc1":["0.36","-0.435"],
"anc2":["0.36","0.435"]
}

DUAL_EKF_PARAMS = os.path.join(os.path.join(get_package_share_directory('uwb_test'),'params'),'dual_ekf_navsat_uwb.yaml')

def generate_launch_description():
    # --- Launch configurations ---
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/lidar')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='false')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    # --- Declare arguments ---
    declare_args = [
        DeclareLaunchArgument('channel_type', default_value='serial',
                              description='Specifying channel type of lidar'),
        DeclareLaunchArgument('serial_port', default_value='/dev/lidar',
                              description='Specifying usb port to connected lidar'),
        DeclareLaunchArgument('serial_baudrate', default_value='256000',
                              description='Specifying usb port baudrate to connected lidar'),
        DeclareLaunchArgument('frame_id', default_value='laser',
                              description='Specifying frame_id of lidar'),
        DeclareLaunchArgument('inverted', default_value='false',
                              description='Specifying whether or not to invert scan data'),
        DeclareLaunchArgument('angle_compensate', default_value='false',
                              description='Specifying whether or not to enable angle compensation'),
        DeclareLaunchArgument('scan_mode', default_value='Sensitivity',
                              description='Specifying scan mode of lidar'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                                             description='Use simulation clock if true'),
		
	# Scout_ros2 parameters
		DeclareLaunchArgument('port_name', default_value='can1',
											 description='CAN bus name, e.g. can1'),
		DeclareLaunchArgument('odom_frame', default_value='odom',
											   description='Odometry frame id'),
		DeclareLaunchArgument('base_frame', default_value='base_link',
													description='Base link frame id'),
		DeclareLaunchArgument('odom_topic_name', default_value='scout/odom',
											   description='Odometry topic name'),

		DeclareLaunchArgument('is_scout_mini', default_value='false',
											  description='Scout mini model'),
		DeclareLaunchArgument('is_omni_wheel', default_value='false',
											  description='Scout mini omni-wheel model'),
		DeclareLaunchArgument('auto_reconnect', default_value='true', 
											  description='Attempts to re-establish CAN command mode'),

		DeclareLaunchArgument('simulated_robot', default_value='false',
													   description='Whether running with simulator'),
		DeclareLaunchArgument('control_rate', default_value='30',
													 description='Simulation control loop update rate'),
													 
	# UWB Anchor locations
		DeclareLaunchArgument('anc0', default_value=STATIC_ANCHORS['anc0'][0]+","+STATIC_ANCHORS['anc0'][1],
													 description='Anchor 1 location (x,y)'),
		DeclareLaunchArgument('anc1', default_value=DYNAMIC_ANCHORS['anc1'][0]+","+DYNAMIC_ANCHORS['anc1'][1],
													 description='Anchor 2 location (x,y)'),
		DeclareLaunchArgument('anc2', default_value=DYNAMIC_ANCHORS['anc2'][0]+","+DYNAMIC_ANCHORS['anc2'][1],
													 description='Anchor 3 location (x,y)'),
		DeclareLaunchArgument('anc3', default_value=STATIC_ANCHORS['anc3'][0]+","+STATIC_ANCHORS['anc3'][1],
													 description='Anchor 4 location (x,y)'),
		DeclareLaunchArgument('anc4', default_value=STATIC_ANCHORS['anc4'][0]+","+STATIC_ANCHORS['anc4'][1],
													 description='Anchor 5 location (x,y)'),
    ]

    # --- RViz config file ---
    rviz_config_dir = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'rviz',
        'rplidar_ros.rviz')

    # --- Nodes ---
    rplidar_ros_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'channel_type': channel_type,
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': frame_id,
            'inverted': inverted,
            'angle_compensate': angle_compensate,
            'scan_mode': scan_mode
        }],
        output='screen'
    )

    umx_driver_node = Node(
        package='umx_driver',
        executable='um7_driver',
        name='um7_node',
        output='screen'
    )

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
    
    ekf_tf_node = Node(
        package='uwb_test',
        executable='ekf_tf',
        name='ekf_tf',
        output='screen'
    )
    
    dynamic_tf_node = Node(
        package='uwb_test',
        executable='dynamic_tf_pub',
        name='dynamic_tf_pub',
        output='screen'
    )

    rviz2_lidar_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )
    
    scout_base_node = Node(
        package='scout_base',
        executable='scout_base_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'port_name': LaunchConfiguration('port_name'),                
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_topic_name': LaunchConfiguration('odom_topic_name'),
                'is_scout_mini': LaunchConfiguration('is_scout_mini'),
                'is_omni_wheel': LaunchConfiguration('is_omni_wheel'),
                'auto_reconnect': LaunchConfiguration('auto_reconnect'),
                'simulated_robot': LaunchConfiguration('simulated_robot'),
                'control_rate': LaunchConfiguration('control_rate'),
        }],
        respawn=True,
        respawn_delay=2.0)
        
    config_directory = os.path.join(get_package_share_directory('ublox_gps'),'config')
    params = os.path.join(config_directory, 'zed_f9p.yaml')
    ublox_gps_node = Node(package='ublox_gps',
							 executable='ublox_gps_node',
							 output='screen',
							 parameters=[params])
    
    static_map_odom = Node(
		package="tf2_ros",
		executable="static_transform_publisher",
		name="static_map_odom",
		arguments=['0.0','0.0','0.0','0.0','0.0','0.0','map','odom']
	)
    
    static_uwb_0 = Node(
		package="tf2_ros",
		executable="static_transform_publisher",
		name="static_uwb_0",
		arguments=[STATIC_ANCHORS['anc0'][0],STATIC_ANCHORS['anc0'][1],'0.0','0.0','0.0','0.0','map_uwb','static_uwb_0']
	)
	
    static_uwb_3 = Node(
		package="tf2_ros",
		executable="static_transform_publisher",
		name="static_uwb_3",
		arguments=[STATIC_ANCHORS['anc3'][0],STATIC_ANCHORS['anc3'][1],'0.0','0.0','0.0','0.0','map_uwb','static_uwb_3']
	)
     
    static_uwb_4 = Node(
		package="tf2_ros",
		executable="static_transform_publisher",
		name="static_uwb_4",
		arguments=[STATIC_ANCHORS['anc4'][0],STATIC_ANCHORS['anc4'][1],'0.0','0.0','0.0','0.0','map_uwb','static_uwb_4']
	)
	
    static_base_imu = Node(
		package="tf2_ros",
		executable="static_transform_publisher",
		name="static_tf_imu",
		arguments=['0.36','0.0','0.0','0.0','0.0','0.0','base_link','imu_link']
	)
	
    static_base_lidar = Node(
		package="tf2_ros",
		executable="static_transform_publisher",
		name="static_tf_lidar",
		arguments=['0.285','0.0','0.0','0.0','3.14','3.14','base_link','laser']
	)
	
    static_base_gnss = Node(
		package="tf2_ros",
		executable="static_transform_publisher",
		name="static_tf_gnss",
		arguments=['0.0','0.0','0.0','0.0','0.0','0.0','base_link','gps']
	)
	
    ekf_filter_node_odom = Node(
	package="robot_localization",
	executable="ekf_node",
	name="ekf_filter_node_odom",
	parameters=[{"ekf_filter_node_odom": ""}, DUAL_EKF_PARAMS],
	remappings=[('odometry/filtered', 'uwb/dynamic_filtered')]
	)
	
    ekf_filter_node_map = Node(
	package="robot_localization",
	executable="ekf_node",
	name="ekf_filter_node_map",
	parameters=[{"ekf_filter_node_map": ""}, DUAL_EKF_PARAMS],
	remappings=[('odometry/filtered', 'uwb/static_filtered')]
	)
	
    delayed_scout = TimerAction(
	period=7.0,
	actions=[scout_base_node]
	)

    # --- Return LaunchDescription ---
    return LaunchDescription(declare_args + [
        delayed_scout,
        static_map_odom,
        static_uwb_0,
        static_uwb_3,
        static_uwb_4,
        static_base_imu,
        static_base_lidar,
        static_base_gnss,
        ublox_gps_node,
        umx_driver_node,
        uwb_rcv_node,
        uwb_tf_node,
        ekf_filter_node_odom,
        ekf_filter_node_map,
        ekf_tf_node,
        #dynamic_tf_node,
        rplidar_ros_node,
        rviz2_lidar_node
    ])
