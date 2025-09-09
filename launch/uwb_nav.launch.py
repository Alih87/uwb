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

    uwb_test_node = Node(
        package='uwb_test',
        executable='uwb_rcv',
        name='uwb_rcv',
        output='screen'
    )

    rviz2_lidar_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    # --- Return LaunchDescription ---
    return LaunchDescription(declare_args + [
        umx_driver_node,
        rplidar_ros_node,
        uwb_test_node,
        rviz2_lidar_node
    ])
