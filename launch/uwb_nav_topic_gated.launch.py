#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

STATIC_ANCHORS = {
    "anc0": ["0.0", "-3.18"],
    "anc3": ["0.0", "0.0"],
    "anc4": ["1.595", "0.175"],
}

DYNAMIC_ANCHORS = {
    "anc1": ["0.495", "-0.6"],
    "anc2": ["1.08", "-0.6"],
}

tag_1 = "tag1"
tag_2 = "tag2"
aux_frame = "uwb"

THIS_DIR = os.path.dirname(os.path.realpath(__file__))
WAIT_SCRIPT = os.path.join(THIS_DIR, "wait_for_topics.py")

DUAL_EKF_PARAMS = os.path.join(get_package_share_directory("uwb_test"), "params", "dual_ekf_navsat_uwb.yaml")
DUAL_EKF_NAVSAT_PARAMS = os.path.join(get_package_share_directory("uwb_test"), "params", "navsat_transform_uwb.yaml")
DUAL_EKF_PARAMS_TAG = os.path.join(get_package_share_directory("uwb_test"), "params", "dual_ekf_navsat_tag.yaml")
DUAL_EKF_PARAMS_TAG1 = os.path.join(get_package_share_directory("uwb_test"), "params", "dual_ekf_navsat_tag1.yaml")
DUAL_EKF_PARAMS_TAG2 = os.path.join(get_package_share_directory("uwb_test"), "params", "dual_ekf_navsat_tag2.yaml")
IMU_PARAMS_TAG1 = os.path.join(get_package_share_directory("uwb_test"), "params", "imu_filter_tag1.yaml")
IMU_PARAMS_TAG2 = os.path.join(get_package_share_directory("uwb_test"), "params", "imu_filter_tag2.yaml")
NAV2_PARAMS = os.path.join(get_package_share_directory("uwb_test"), "params", "nav2_params.yaml")


def make_gate(stage_name, specs, stable_for):
    cmd = [
        "python3",
        WAIT_SCRIPT,
        "--stage-name",
        stage_name,
        "--stable-for",
        str(stable_for),
        "--timeout",
        "0",
        "--status-every",
        "5.0",
    ]
    for spec in specs:
        cmd.extend(["--spec", spec])
    return ExecuteProcess(cmd=cmd, output="screen")


def generate_launch_description():
    rplidar_channel_type = LaunchConfiguration("rplidar_channel_type")
    rplidar_serial_port = LaunchConfiguration("rplidar_serial_port")
    rplidar_serial_baudrate = LaunchConfiguration("rplidar_serial_baudrate")
    rplidar_frame_id = LaunchConfiguration("rplidar_frame_id")
    rplidar_inverted = LaunchConfiguration("rplidar_inverted")
    rplidar_angle_compensate = LaunchConfiguration("rplidar_angle_compensate")
    rplidar_scan_mode = LaunchConfiguration("rplidar_scan_mode")

    nav2_use_sim_time = LaunchConfiguration("nav2_use_sim_time")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    nav2_autostart = LaunchConfiguration("nav2_autostart")

    declare_args = [
        DeclareLaunchArgument("rplidar_channel_type", default_value="serial"),
        DeclareLaunchArgument("rplidar_serial_port", default_value="/dev/lidar"),
        DeclareLaunchArgument("rplidar_serial_baudrate", default_value="256000"),
        DeclareLaunchArgument("rplidar_frame_id", default_value="laser"),
        DeclareLaunchArgument("rplidar_inverted", default_value="false"),
        DeclareLaunchArgument("rplidar_angle_compensate", default_value="false"),
        DeclareLaunchArgument("rplidar_scan_mode", default_value="Sensitivity"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("scout_use_sim_time", default_value="false"),
        DeclareLaunchArgument("scout_port_name", default_value="can1"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("odom_topic_name", default_value="scout/odom"),
        DeclareLaunchArgument("is_scout_mini", default_value="false"),
        DeclareLaunchArgument("is_omni_wheel", default_value="false"),
        DeclareLaunchArgument("auto_reconnect", default_value="true"),
        DeclareLaunchArgument("simulated_robot", default_value="false"),
        DeclareLaunchArgument("control_rate", default_value="30"),
        DeclareLaunchArgument("nav2_use_sim_time", default_value="false"),
        DeclareLaunchArgument("nav2_autostart", default_value="true"),
        DeclareLaunchArgument("nav2_params_file", default_value=NAV2_PARAMS),
    ]

    rviz_config_dir = os.path.join(get_package_share_directory("rplidar_ros"), "rviz", "rplidar_ros.rviz")
    rviz2_config = os.path.join(get_package_share_directory("uwb_test"), "config", "rviz2_config_.rviz")

    # Optional sensors / extras kept from your original launch
    rplidar_ros_node = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[{
            "channel_type": rplidar_channel_type,
            "serial_port": rplidar_serial_port,
            "serial_baudrate": rplidar_serial_baudrate,
            "frame_id": rplidar_frame_id,
            "inverted": rplidar_inverted,
            "angle_compensate": rplidar_angle_compensate,
            "scan_mode": rplidar_scan_mode,
        }],
        output="screen",
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("uwb_test"), "launch", "navigation_uwb_launch.py")
        ),
        launch_arguments={
            "use_sim_time": nav2_use_sim_time,
            "params_file": nav2_params_file,
            "autostart": nav2_autostart,
            "use_composition": "False",
        }.items(),
    )

    baselink_transformer = Node(
        package="imu_transformer",
        executable="imu_transformer_node",
        name="baselink_transformer_node",
        output="screen",
        remappings=[("imu_in", "/imu/data_raw"), ("imu_out", "/imu/data")],
        parameters=[{"target_frame": "base_link"}],
        respawn=True,
        respawn_delay=3.0,
    )

    umx_driver_node = Node(
        package="umx_driver",
        executable="um7_driver",
        name="um7_node",
        output="screen",
        remappings=[("imu/data", "imu/data_raw")],
        respawn=True,
        respawn_delay=3.0,
    )

    uwb_rcv_single_node = Node(
        package="uwb_test",
        executable="uwb_rcv",
        name="uwb_rcv_single",
        output="screen",
    )

    realsense_launch = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        name="camera",
        output="screen",
        parameters=[{
            "enable_color": True,
            "enable_depth": True,
            "enable_sync": True,
            "align_depth.enable": True,
            "pointcloud__neon_.enable": True,
        }],
        arguments=["--ros-args", "--log-level", "info"],
    )

    dynamic_tf_node = Node(
        package="uwb_test",
        executable="dynamic_tf_pub",
        name="dynamic_tf_pub",
        output="screen",
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz2_config],
    )

    rviz2_lidar_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_lidar",
        arguments=["-d", rviz_config_dir],
        output="screen",
    )

    scout_base_node = Node(
        package="scout_base",
        executable="scout_base_node",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "use_sim_time": LaunchConfiguration("scout_use_sim_time"),
            "port_name": LaunchConfiguration("scout_port_name"),
            "odom_frame": LaunchConfiguration("odom_frame"),
            "base_frame": LaunchConfiguration("base_frame"),
            "odom_topic_name": LaunchConfiguration("odom_topic_name"),
            "is_scout_mini": LaunchConfiguration("is_scout_mini"),
            "is_omni_wheel": LaunchConfiguration("is_omni_wheel"),
            "auto_reconnect": LaunchConfiguration("auto_reconnect"),
            "simulated_robot": LaunchConfiguration("simulated_robot"),
            "control_rate": LaunchConfiguration("control_rate"),
        }],
        respawn=True,
        respawn_delay=3.0,
    )

    ublox_config_directory = os.path.join(get_package_share_directory("ublox_gps"), "config")
    ublox_params = os.path.join(get_package_share_directory("uwb_test"), "params", "zed_f9p.yaml")
    ublox_gps_node = Node(
        package="ublox_gps",
        executable="ublox_gps_node",
        output="screen",
        parameters=[ublox_params],
        respawn=True,
        respawn_delay=3.0,
    )
    
    static_base_camera = Node(
		package="tf2_ros",
		executable="static_transform_publisher",
		name="static_tf_camera",
		arguments=['0.33','-0.048','0.39','0.0','0.0','0.0','base_link','camera_link']
	)
    
    static_base_tag = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='static_tf_tag',
		arguments=[
			'--x', '0.39', '--y', '0.025', '--z', '0.33',
			'--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
			'--frame-id', 'base_link',
			'--child-frame-id', 'tag_link',
		],
	)

    static_base_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_imu",
        arguments=[
            "--x", "0.26", "--y", "0.0", "--z", "0.39",
            "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
            "--frame-id", "base_link",
            "--child-frame-id", "imu_link",
        ],
    )

    static_base_gnss = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_gnss",
        arguments=["0.0", "0.0", "0.365", "0.0", "0.0", "0.0", "base_link", "gps"],
    )
    
    tag_ekf_launch = IncludeLaunchDescription(
            launch_description_source=os.path.join(os.path.join(get_package_share_directory('uwb_test'),'launch'),'tag_ekf.launch.py'),
            launch_arguments={
                'anc0': STATIC_ANCHORS['anc0'][0]+","+STATIC_ANCHORS['anc0'][1],
                'anc1': DYNAMIC_ANCHORS['anc1'][0]+","+DYNAMIC_ANCHORS['anc1'][1],
                'anc2': DYNAMIC_ANCHORS['anc2'][0]+","+DYNAMIC_ANCHORS['anc2'][1],
                'anc3': STATIC_ANCHORS['anc3'][0]+","+STATIC_ANCHORS['anc3'][1],
                'anc4': STATIC_ANCHORS['anc4'][0]+","+STATIC_ANCHORS['anc4'][1],
                'tag_frame': "",
                'aux_frame': aux_frame,
                'ekf_params': DUAL_EKF_PARAMS_TAG
            }.items()
        )

    ekf_filter_node_fused = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_fused",
        parameters=[{"ekf_filter_node_odom": ""}, DUAL_EKF_PARAMS],
        remappings=[("odometry/filtered", "scout/odom_filtered")],
        output="screen",
    )

    navsat_node_ = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[DUAL_EKF_NAVSAT_PARAMS],
        remappings=[
            ("/odometry/filtered", "/scout/odom_filtered"),
            ("/gps/fix", "/ublox_gps_node/fix"),
            ("/imu", "/imu/data"),
        ],
    )
    
    odom_to_gps_frame = Node(
    package="uwb_test",
    executable="gps_odom_to_map_frame",
    name="odom_to_gps_frame",
    output="screen",
    parameters=[{
        "input_topic": "/odometry/gps",
        "output_topic": "/odometry/gps_map",
        "frame_id": "map",
        "child_frame_id": "base_link",
        "xy_variance_floor": 0.01,
        "z_variance_floor": 0.04,
    }]
)

    ekf_filter_node_map = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        parameters=[{"ekf_filter_node_map": ""}, DUAL_EKF_PARAMS],
        remappings=[("odometry/filtered", "scout/map")],
        output="screen",
    )

    # Gates: infinite patience for flaky hardware; stricter stability before EKF stages.
    gate_hardware = make_gate(
        "Hardware stable",
        [
            "/scout/odom|nav_msgs/msg/Odometry|5|2.0",
            "/imu/data|sensor_msgs/msg/Imu|10|1.0",
            "/ublox_gps_node/fix|sensor_msgs/msg/NavSatFix|3|3.0",
        ],
        stable_for=5.0,
    )

    gate_local_ekf = make_gate(
        "Local EKF stable",
        ["/scout/odom_filtered|nav_msgs/msg/Odometry|5|1.5"],
        stable_for=3.0,
    )

    gate_navsat = make_gate(
        "NavSat stable",
        ["/odometry/gps|nav_msgs/msg/Odometry|3|3.0"],
        stable_for=3.0,
    )

    gate_global_ekf = make_gate(
        "Global EKF stable",
        ["/scout/map|nav_msgs/msg/Odometry|3|3.0"],
        stable_for=3.0,
    )
    
    gate_gps_map = make_gate(
    "GPS map odom stable",
    ["/odometry/gps_map|nav_msgs/msg/Odometry|3|3.0"],
    stable_for=3.0,
)

    return LaunchDescription(declare_args + [
        # Static TFs
        static_base_imu,
        static_base_gnss,
        #static_base_camera,
        #static_base_tag,

        scout_base_node,
        ublox_gps_node,
        umx_driver_node,
        baselink_transformer,

        # rplidar_ros_node,
        #uwb_rcv_single_node,
        #tag_ekf_launch,
        # realsense_launch,
        # dynamic_tf_node,

        LogInfo(msg="[stage] hardware started; waiting indefinitely for stable Scout/GPS/IMU topics..."),
        gate_hardware,

        RegisterEventHandler(
            OnProcessExit(
                target_action=gate_hardware,
                on_exit=[
                    LogInfo(msg="[stage] hardware stable -> starting local EKF"),
                    ekf_filter_node_fused,
                    gate_local_ekf,
                ],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=gate_local_ekf,
                on_exit=[
                    LogInfo(msg="[stage] local EKF stable -> starting navsat_transform"),
                    navsat_node_,
                    gate_navsat,
                ],
            )
        ),
        RegisterEventHandler(
			OnProcessExit(
				target_action=gate_navsat,
				on_exit=[
					LogInfo(msg="[stage] navsat stable -> starting odom_to_gps_frame"),
					odom_to_gps_frame,
					gate_gps_map,
				],
			)
		),
       RegisterEventHandler(
			OnProcessExit(
				target_action=gate_gps_map,
				on_exit=[
					LogInfo(msg="[stage] gps_map stable -> starting global EKF"),
					ekf_filter_node_map,
					nav2_bringup_launch,
					gate_global_ekf,
				],
			)
		),
    ])


if __name__ == "__main__":
    generate_launch_description()
