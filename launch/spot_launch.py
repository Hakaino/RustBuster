# Import necessary ROS2 packages
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, actions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	# Get configuration files
	map_file = os.path.join(get_package_share_directory('rustbuster'), 'maps/map.yaml')

	# Create launch description
	ld = LaunchDescription()
	ld.add_action(DeclareLaunchArgument('use_sim_time', default_value="False", description='RustBuster main launcher'))

	# Ros2 bag
	if 0:
		bag_path = "rosbag2_2023_04_12-14_17_02"
		ld.add_action(actions.ExecuteProcess(
				cmd=['ros2', 'bag', 'play', bag_path
					, "--topics", "/odometry", "/points_back", "/points_left", "/points_right", "/points_frontleft"
					, "/points_frontright", "/camera/back/image", "/robot_description", "/tf", "/tf_static"],
					#, "/depth/frontleft/image", "/camera/frontleft/camera_info" ],
				output='log'))
		#ld.add_action(actions.ExecuteProcess( cmd=['ros2', 'bag', 'record', "--all"], output='log' ))

	# Nav2
	if 1:
		nav2_launch = os.path.join(get_package_share_directory("nav2_bringup"), 'launch', "navigation_launch.py")
		ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(nav2_launch)))

	# My controller
	if 1:
		ld.add_action(Node(
			package="rustbuster",
			executable="rustbuster_init",
			name='rustbuster_main',
			output='screen',
		))

	# RVIZ2
	if 1:
		ld.add_action(Node(
				package='rviz2',
				namespace='rviz2',
				executable='rviz2',
				name='rviz2',
				arguments=["-d", os.path.join(get_package_share_directory("rustbuster"), "config", "spot_rviz2_config.rviz")],
				output='log'
		))

	# Spot driver
	if 1:
		spot_config = os.path.join(get_package_share_directory('rustbuster'), 'config/spot_config.yaml')
		spot_launch = os.path.join(get_package_share_directory('spot_driver'), 'launch', "spot_driver.launch.py")
		ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(spot_launch),
		                                       launch_arguments={"config_file": spot_config}.items()))

		ld.add_action(Node(  # this can replace a lot
				package='tf2_ros',
				executable='static_transform_publisher',
				arguments=['2.5', '2.5', '0', '0', '0', '0', '1', 'map', 'odom']
		))

		ld.add_action(Node(  # this can replace a lot
				package='tf2_ros',
				executable='static_transform_publisher',
				arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'body']
		))

		ld.add_action(Node(  # this can replace a lot
				package='tf2_ros',
				executable='static_transform_publisher',
				arguments=['0', '0', '0', '0', '0', '0', '1', 'body', 'imu']
		))

	# point_cloud_xyz
	if 1:
		depth_to_launch = os.path.join(get_package_share_directory("spot_driver"), 'launch', "point_cloud_xyz.launch.py")
		ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(depth_to_launch)))

	# Point cloud to laser scan
	if 0:
		ld.add_action(Node(
			package='pointcloud_to_laserscan',
			executable='pointcloud_to_laserscan_node',
			name='pointcloud_to_laserscan_node',
			remappings=[('cloud_in', '/points_back'), ('scan', '/scan')],
			parameters=[{
				'target_frame': 'body',
				'transform_tolerance': 0.01,
				'min_height': 0.0,
				'max_height': 1.0,
				'angle_min': -1.5708,  # -M_PI/2
				'angle_max': 1.5708,  # M_PI/2
				'angle_increment': 0.0087,  # M_PI/360.0
				'scan_time': 0.3333,
				'range_min': 0.3,
				'range_max': 2.0,
				'use_inf': True,
				'inf_epsilon': 1.0
			}],
		))

	# cartographer
	if 1:
		ld.add_action(Node(
				package='cartographer_ros',
				executable='cartographer_node',
				arguments=[
					'-configuration_directory',
					get_package_share_directory('rustbuster') + '/config', '-configuration_basename', "backpack_3d.lua"],
				remappings=[
					('points2_1', 'points_frontright'),
					('points2_2', 'points_frontleft'),
					('points2_3', 'points_right'),
					('points2_4', 'points_left'),
					('points2_5', 'points_back'),
					#('base_link', 'spot_base_link'),
					#('imu', 'camera/imu'),
				],
				output='screen'
		))

		ld.add_action(Node(
				package='cartographer_ros',
				executable='cartographer_occupancy_grid_node',
				parameters=[{'resolution': 0.05}],
		))

	# Explorer
	if 0:
		explore_launch = os.path.join(get_package_share_directory("explore_lite"), 'launch', "explore.launch.py")
		ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(explore_launch), launch_arguments={"headless": "true"}.items()
		))

	"""going another way"""
	# rtabmaps
	if 0:
		"""
		ld.add_action(DeclareLaunchArgument("rtabmap_args", default_value="--delete_db_on_start"))
		ld.add_action(DeclareLaunchArgument("rgb_topic", default_value="/camera/color/image_raw"))
		ld.add_action(DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_rect_raw"))
		ld.add_action(DeclareLaunchArgument("camera_info_topic", default_value="/camera/color/camera_info"))
		ld.add_action(DeclareLaunchArgument("approx_sync", default_value="true"))"""

		rtabmap_launch = os.path.join(get_package_share_directory('rtabmap_ros'), 'launch', "realsense_d400.launch.py")
		ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(rtabmap_launch)))

		#rtabmap_args := "--delete_db_on_start" \
		#rgb_topic := / kinect2 / qhd / image_color_rect \
		#depth_topic := / kinect2 / qhd / image_depth_rect \
		#camera_info_topic := / kinect2 / qhd / camera_info \
		#approx_sync := false

	# spot_description
	if 0:
		spot_launch = os.path.join(get_package_share_directory('spot_description'), 'launch', "description.launch.py")
		ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(spot_launch)))

	# Realsense
	if 0:
		realsense_launch = os.path.join(get_package_share_directory("realsense2_camera"), 'launch', "rs_launch.py")
		ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(realsense_launch)))

	# slam toolbox
	if 0:
		ld.add_action(Node(
				parameters=[os.path.join(get_package_share_directory('rustbuster'), 'config/exploration_params.yaml')],
				package='slam_toolbox',
				executable='async_slam_toolbox_node',
				name='slam_toolbox',
				output='screen'))

	# Ouster
	if 0:
		ouster_launch = os.path.join(get_package_share_directory("ros2_ouster"), 'launch', "driver_launch.py")
		ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(ouster_launch)))

	# lidarslam
	if 0:
		ld.add_action(Node(
				package='scanmatcher',
				executable='scanmatcher_node',
				parameters=[LaunchConfiguration('main_param_dir',
						default=os.path.join(get_package_share_directory('lidarslam'), 'param', 'lidarslam.yaml'))],
				remappings=[('/input_cloud', '/points_frontleft')],
				output='screen'
		))


		ld.add_action(Node(
				package='graph_based_slam',
				executable='graph_based_slam_node',
				parameters=[os.path.join(get_package_share_directory('lidarslam'), 'param', 'lidarslam.yaml')],
				output='screen'
		))

	return ld