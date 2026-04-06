from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os
from launch.actions import TimerAction

STATIC_ANCHORS = {
"anc0":["0.0","-3.18"],
"anc3":["0.0","0.0"],
"anc4":["1.595","0.175"]
}

DYNAMIC_ANCHORS = {
"anc1":["0.495","-0.6"],
#"anc2":["0.36","0.435"]
"anc2":["1.08","-0.6"]
}

tag_1 = "tag1"
tag_2 = "tag2"

aux_frame = "utm"

DUAL_EKF_PARAMS = os.path.join(os.path.join(get_package_share_directory('uwb_test'),'params'),'dual_ekf_navsat_uwb.yaml')
DUAL_EKF_NAVSAT_PARAMS = os.path.join(os.path.join(get_package_share_directory('uwb_test'),'params'),'navsat_transform_uwb.yaml')
DUAL_EKF_PARAMS_TAG1 = os.path.join(os.path.join(get_package_share_directory('uwb_test'),'params'),'dual_ekf_navsat_tag1.yaml')
DUAL_EKF_PARAMS_TAG2 = os.path.join(os.path.join(get_package_share_directory('uwb_test'),'params'),'dual_ekf_navsat_tag2.yaml')
IMU_PARAMS_TAG1 = os.path.join(os.path.join(get_package_share_directory('uwb_test'),'params'),'imu_filter_tag1.yaml')
IMU_PARAMS_TAG2 = os.path.join(os.path.join(get_package_share_directory('uwb_test'),'params'),'imu_filter_tag2.yaml')
NAV2_PARAMS = os.path.join(os.path.join(get_package_share_directory('uwb_test'),'params'),'nav2_params.yaml')

def generate_launch_description():
    # --- Launch configurations ---
    rplidar_channel_type = LaunchConfiguration('rplidar_channel_type')
    rplidar_serial_port = LaunchConfiguration('rplidar_serial_port')
    rplidar_serial_baudrate = LaunchConfiguration('rplidar_serial_baudrate')
    rplidar_frame_id = LaunchConfiguration('rplidar_frame_id')
    rplidar_inverted = LaunchConfiguration('rplidar_inverted')
    rplidar_angle_compensate = LaunchConfiguration('rplidar_angle_compensate')
    rplidar_scan_mode = LaunchConfiguration('rplidar_scan_mode')
    
    
    nav2_use_sim_time = LaunchConfiguration('nav2_use_sim_time')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    nav2_autostart = LaunchConfiguration('nav2_autostart')

    # --- Declare arguments ---
    # rplidar parameters
    declare_args = [
        DeclareLaunchArgument('rplidar_channel_type', default_value='serial',
                              description='Specifying channel type of lidar'),
        DeclareLaunchArgument('rplidar_serial_port', default_value='/dev/lidar',
                              description='Specifying usb port to connected lidar'),
        DeclareLaunchArgument('rplidar_serial_baudrate', default_value='256000',
                              description='Specifying usb port baudrate to connected lidar'),
        DeclareLaunchArgument('rplidar_frame_id', default_value='laser',
                              description='Specifying frame_id of lidar'),
        DeclareLaunchArgument('rplidar_inverted', default_value='false',
                              description='Specifying whether or not to invert scan data'),
        DeclareLaunchArgument('rplidar_angle_compensate', default_value='false',
                              description='Specifying whether or not to enable angle compensation'),
        DeclareLaunchArgument('rplidar_scan_mode', default_value='Sensitivity',
                              description='Specifying scan mode of lidar'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                                             description='Use simulation clock if true'),
		
	# Scout_ros2 parameters
		DeclareLaunchArgument('scout_use_sim_time', default_value='false',
                                             description='Use simulation clock if true'),
		DeclareLaunchArgument('scout_port_name', default_value='can1',
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
													 
	# Navigation2 parameters												 
		DeclareLaunchArgument('nav2_use_sim_time', default_value='false'),
        DeclareLaunchArgument('nav2_autostart', default_value='true'),
        DeclareLaunchArgument('nav2_params_file', default_value=NAV2_PARAMS),
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
            'channel_type': rplidar_channel_type,
            'serial_port': rplidar_serial_port,
            'serial_baudrate': rplidar_serial_baudrate,
            'frame_id': rplidar_frame_id,
            'inverted': rplidar_inverted,
            'angle_compensate': rplidar_angle_compensate,
            'scan_mode': rplidar_scan_mode
        }],
        output='screen'
    )
    
    nav2_bringup_launch = IncludeLaunchDescription(
		launch_description_source=os.path.join(os.path.join(get_package_share_directory('uwb_test'),'launch'),'navigation_uwb_launch.py'),
		launch_arguments={
			'use_sim_time': nav2_use_sim_time,
			'params_file': nav2_params_file,
			'autostart': nav2_autostart,
			'use_composition': "False"
		}.items()
	)
    delayed_nav2 = TimerAction(
		period=12.0,
		actions=[nav2_bringup_launch]
	)
    
    baselink_transformer = Node(
		package='imu_transformer',
		executable='imu_transformer_node',
		name='baselink_transformer_node',
		output='screen',
		remappings=[
			('imu_in',  '/imu/data_raw'),
			('imu_out', '/imu/data'),
		],
		parameters=[{'target_frame': 'base_link'}],
	)
    
    umx_driver_node = Node(
        package='umx_driver',
        executable='um7_driver',
        name='um7_node',
        output='screen',
        remappings=[('imu/data', 'imu/data_raw')]
    )
    
    uwb_rcv_node = Node(
        package='uwb_test',
        executable='uwb_rcv',
        name='uwb_rcv',
        parameters=[{
			'tag1': tag_1,
			'tag2': tag_2
        }],
        output='screen'
    )
    
    realsense_launch = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera',
        name='camera',
        output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'align_depth.enable': True,
            'pointcloud__neon_.enable': True,
        }],
        arguments=['--ros-args', '--log-level', 'info'],
    )
    
    dynamic_tf_node = Node(
        package='uwb_test',
        executable='dynamic_tf_pub',
        name='dynamic_tf_pub',
        output='screen'
    )
    
    rviz2_config = os.path.join(
        get_package_share_directory('uwb_test'),
        'config',
        'rviz2_config_.rviz'
    )   
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz2_config]
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
                'use_sim_time': LaunchConfiguration('scout_use_sim_time'),
                'port_name': LaunchConfiguration('scout_port_name'),                
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
							 parameters=[params], respawn=True, respawn_delay=2.0)
    
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
		arguments=[STATIC_ANCHORS['anc0'][0],STATIC_ANCHORS['anc0'][1],'0.0','0.0','0.0','0.0',aux_frame,'static_uwb_0']
	)
	
    static_uwb_3 = Node(
		package="tf2_ros",
		executable="static_transform_publisher",
		name="static_uwb_3",
		arguments=[STATIC_ANCHORS['anc3'][0],STATIC_ANCHORS['anc3'][1],'0.0','0.0','0.0','0.0',aux_frame,'static_uwb_3']
	)
     
    static_uwb_4 = Node(
		package="tf2_ros",
		executable="static_transform_publisher",
		name="static_uwb_4",
		arguments=[STATIC_ANCHORS['anc4'][0],STATIC_ANCHORS['anc4'][1],'0.0','0.0','0.0','0.0',aux_frame,'static_uwb_4']
	)

    static_base_imu = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='static_tf_imu',
		arguments=[
			'--x', '-0.36', '--y', '0.0', '--z', '0.0',
			'--qx', '0.70710678', '--qy', '0.70710678', '--qz', '0', '--qw', '0',
			'--frame-id', 'base_link',
			'--child-frame-id', 'imu_link',
		],
	)

    static_base_camera = Node(
		package="tf2_ros",
		executable="static_transform_publisher",
		name="static_tf_camera",
		arguments=['0.285','-0.075','0.0','0.0','0.0','0.0','base_link','camera_link']
	)
	
    static_base_gnss = Node(
		package="tf2_ros",
		executable="static_transform_publisher",
		name="static_tf_gnss",
		arguments=['0.0','0.0','0.0','0.0','0.0','0.0','base_link','gps']
	)

    ekf_filter_node_fused = Node(
		package="robot_localization",
		executable="ekf_node",
		name='ekf_filter_node_fused',
		parameters=[{"ekf_filter_node_odom": ""}, DUAL_EKF_PARAMS],
		remappings=[('odometry/filtered', 'scout/odom_filtered')]
	)

    navsat_node_ = Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[DUAL_EKF_NAVSAT_PARAMS],
            remappings=[('odometry/filtered', 'scout/odom_filtered'),
						('gps/fix', '/ublox_gps_node/fix'),
						('/imu', '/imu/data')]
    )
	
    ekf_filter_node_map = Node(
		package="robot_localization",
		executable="ekf_node",
		name='ekf_filter_node_map',
		parameters=[{"ekf_filter_node_map": ""}, DUAL_EKF_PARAMS],
		remappings=[('odometry/filtered', 'scout/map')]
	)
	
    tag1_ekf_launch = IncludeLaunchDescription(
            launch_description_source=os.path.join(os.path.join(get_package_share_directory('uwb_test'),'launch'),'tag_ekf.launch.py'),
            launch_arguments={
                'anc0': STATIC_ANCHORS['anc0'][0]+","+STATIC_ANCHORS['anc0'][1],
                'anc1': DYNAMIC_ANCHORS['anc1'][0]+","+DYNAMIC_ANCHORS['anc1'][1],
                'anc2': DYNAMIC_ANCHORS['anc2'][0]+","+DYNAMIC_ANCHORS['anc2'][1],
                'anc3': STATIC_ANCHORS['anc3'][0]+","+STATIC_ANCHORS['anc3'][1],
                'anc4': STATIC_ANCHORS['anc4'][0]+","+STATIC_ANCHORS['anc4'][1],
                'tag_frame': tag_1,
                'aux_frame': aux_frame,
                'ekf_params': DUAL_EKF_PARAMS_TAG1,
                'imu_params': IMU_PARAMS_TAG1
            }.items()
        )
        
    tag2_ekf_launch = IncludeLaunchDescription(
            launch_description_source=os.path.join(os.path.join(get_package_share_directory('uwb_test'),'launch'),'tag_ekf.launch.py'),
            launch_arguments={
                'anc0': STATIC_ANCHORS['anc0'][0]+","+STATIC_ANCHORS['anc0'][1],
                'anc1': DYNAMIC_ANCHORS['anc1'][0]+","+DYNAMIC_ANCHORS['anc1'][1],
                'anc2': DYNAMIC_ANCHORS['anc2'][0]+","+DYNAMIC_ANCHORS['anc2'][1],
                'anc3': STATIC_ANCHORS['anc3'][0]+","+STATIC_ANCHORS['anc3'][1],
                'anc4': STATIC_ANCHORS['anc4'][0]+","+STATIC_ANCHORS['anc4'][1],
                'tag_frame': tag_2,
                'aux_frame': aux_frame,
                'ekf_params': DUAL_EKF_PARAMS_TAG2,
                'imu_params': IMU_PARAMS_TAG2
            }.items()
        )
        
    delayed_scout = TimerAction(
		period=7.0,
		actions=[scout_base_node]
	)

    # --- Return LaunchDescription ---
    return LaunchDescription(declare_args + [
        delayed_scout,
        #static_map_odom,
        #static_uwb_0,
        #static_uwb_3,
        #static_uwb_4,
        static_base_imu,
        static_base_camera,
        static_base_gnss,
        ublox_gps_node,
        umx_driver_node,
        baselink_transformer,
        realsense_launch,
        ekf_filter_node_fused,
        navsat_node_,
        ekf_filter_node_map,
        #delayed_nav2,
        #uwb_rcv_node,
        #tag1_ekf_launch,
        #tag2_ekf_launch,
        #rplidar_ros_node,
        #rviz2_lidar_node,
        rviz2_node
    ])
