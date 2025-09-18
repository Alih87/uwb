from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # --- Launch configurations ---
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/lidar')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
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
        DeclareLaunchArgument('angle_compensate', default_value='true',
                              description='Specifying whether or not to enable angle compensation'),
        DeclareLaunchArgument('scan_mode', default_value='Sensitivity',
                              description='Specifying scan mode of lidar'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                                             description='Use simulation clock if true'),
		
	# Scout_ros2 parameters
		DeclareLaunchArgument('port_name', default_value='can0',
											 description='CAN bus name, e.g. can0'),
		DeclareLaunchArgument('odom_frame', default_value='odom',
											   description='Odometry frame id'),
		DeclareLaunchArgument('base_frame', default_value='base_link',
													description='Base link frame id'),
		DeclareLaunchArgument('odom_topic_name', default_value='odom',
											   description='Odometry topic name'),

		DeclareLaunchArgument('is_scout_mini', default_value='false',
											  description='Scout mini model'),
		DeclareLaunchArgument('is_omni_wheel', default_value='false',
											  description='Scout mini omni-wheel model'),
		DeclareLaunchArgument('auto_reconnect', default_value='true', 
											  description='Attempts to re-establish CAN command mode'),

		DeclareLaunchArgument('simulated_robot', default_value='false',
													   description='Whether running with simulator'),
		DeclareLaunchArgument('control_rate', default_value='50',
													 description='Simulation control loop update rate')
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
        }])

    # --- Return LaunchDescription ---
    return LaunchDescription(declare_args + [
        umx_driver_node,
        rplidar_ros_node,
        uwb_rcv_node,
        uwb_tf_node,
        scout_base_node,
        rviz2_lidar_node
    ])
