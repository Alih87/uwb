from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
	
    slam_parameters={
          'frame_id':'base_link',
          'subscribe_depth':True,
          'subscribe_odom_info':False,
          'approx_sync':True}

    slam_remappings=[
          ('imu', '/imu/data'),
          ('odom','/scout/odom_filtered'),
          ('rgb/image', '/camera/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/camera/color/camera_info'),
          ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw')]
          
    map_rtab_node = Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[slam_parameters,
            {
            'subscribe_depth': True,
            'subscribe_scan': False,
            'queue_size': 20,
			'Grid/Sensor': '1',
			'Grid/DepthDecimation': '4',
			'Grid/CellSize': '0.05',
			'Grid/3D': 'false',
			'Grid/RangeMin': '0.3',
			'Grid/RangeMax': '4.0',
			'RGBD/LinearUpdate': '0.05',
			'RGBD/AngularUpdate': '0.05',
			'RGBD/OptimizeMaxError': '0.1',
			'Rtabmap/DetectionRate': '2.0',
            'database_path': os.path.join(os.path.join(get_package_share_directory('uwb_test'),'config'),'rtab_map_corridor.db'),
            'Mem/IncrementalMemory': 'False',
			'Mem/InitWMWithAllNodes': 'True',
			'frame_id': 'base_link',
			'odom_frame_id': 'odom',
			'publish_tf': True
			}
				],
            remappings=slam_remappings)
          
    return LaunchDescription([
		map_rtab_node
    ])
