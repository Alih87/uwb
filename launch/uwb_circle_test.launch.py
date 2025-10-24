from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
    
    # --- Return LaunchDescription ---
    return LaunchDescription([
        uwb_rcv_node,
        uwb_tf_node
    ])
