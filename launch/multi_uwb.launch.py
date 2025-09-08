from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package="uwb_test",
			executable="uwb_node",
			name="uwb0_node",
			arguments=["115200","1000","/dev/uwb0"]
		),
		Node(
			package="uwb_test",
			executable="uwb_node",
			name="uwb1_node",
			arguments=["115200","1000","/dev/uwb1"]
		),
		Node(
			package="uwb_test",
			executable="uwb_node",
			name="uwb2_node",
			arguments=["115200","1000","/dev/uwb2"]
		),
	])
