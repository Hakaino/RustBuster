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
	ld.add_action(DeclareLaunchArgument('use_sim_time', default_value = "False", description='to change when using simulations or bag files'))
	#ld.add_action(Node(package='tf2_ros', executable='static_transform_publisher',
	#		arguments=['.0', '.0', '0.0', '0.0', '0.0', '0.0' 'map', 'camera_link']))
			#arguments=['2.0', '2.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']))
	#ld.add_action(Node(package='tf2_ros', executable='static_transform_publisher',
			#arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'body']))
	#ld.add_action(Node(package='tf2_ros', executable='static_transform_publisher',
			#arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'body', 'imu']))

	""" """
	# Ros2 bag
	if 1:
		bag_path = "rosbag2_2023_04_26-15_36_29"
		ld.add_action(actions.ExecuteProcess(cmd=['ros2', 'bag', 'play', bag_path]))
					#, "--topics", "/odometry", "/points_back", "/points_left", "/points_right", "/points_frontleft"
					#, "/points_frontright", "/camera/back/image", "/robot_description", "/tf", "/tf_static"],
					#, "/depth/frontleft/image", "/camera/frontleft/camera_info" ],
					# output='log'))
		#ld.add_action(actions.ExecuteProcess( cmd=['ros2', 'bag', 'record', "--all"], output='log' ))

	# Nav2
	if 0:
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

	# Spot driver
	if 0:
		spot_config = os.path.join(get_package_share_directory('rustbuster'), 'config/spot_config.yaml')
		spot_launch = os.path.join(get_package_share_directory('spot_driver'), 'launch', "spot_driver.launch.py")
		ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(spot_launch),
		                                       launch_arguments={"config_file": spot_config}.items()))

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

	# point_cloud_xyz
	if 0:
		depth_to_launch = os.path.join(get_package_share_directory("spot_driver"), 'launch', "point_cloud_xyz.launch.py")
		ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(depth_to_launch)))

	# ... to laser scan
	if 0:
		ld.add_action(Node(
			package= "depthimage_to_laserscan",  #'pointcloud_to_laserscan',
			executable= "depthimage_to_laserscan_node",  # "'pointcloud_to_laserscan_node',
			name='laserscan_converter_node',
			remappings=[('depth', '/camera/aligned_depth_to_color/image_raw'), ('depth_camera_info', 'camera/depth/camera_info')],
			#remappings=[('cloud_in', '/points_left'), ('scan', '/scan')],
			parameters=[{
				"scan_time": "0.5",
				"range_min": "0.1",
				"range_max": "10.0",
				"scan_height": "0.5",
				"output_frame": "camera_link"

				#'target_frame': 'left_fisheye',
				#'transform_tolerance': 0.01,
				#'min_height': 0.1,
				#'max_height': 1.0,
				#'angle_min': -1.5708,  # -M_PI/2
				#'angle_max': 1.5708,  # M_PI/2
				#'angle_increment': 0.0087,  # M_PI/360.0
				#'scan_time': 0.3333,
				#'range_min': 0.0,
				#'range_max': 40.0,
				#'use_inf': True,
				#'inf_epsilon': 1.0
			}],
		))

	# cartographer
	if 0:
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
	# rtabmaps (commented out parts to find the bug in the installation)
	if 1:
		parameters = [{
			'frame_id': 'camera_link',
			'subscribe_depth': True,
			'approx_sync': True,
			'wait_imu_to_init': True,
            'subscribe_rgb':True,
            'subscribe_scan':False,
			'use_action_for_goal':True,
            'qos_scan':2,
	        'qos_imu':2,
	        'Reg/Strategy':'1',
	        'Reg/Force3DoF':'true',
	        'RGBD/NeighborLinkRefining':'True',
	        'Grid/RangeMin':'0.2', # ignore laser scan points on the robot itself
	        'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
		}]

		remappings = [
			('imu', '/imu/data'),
			('rgb/image', '/camera/color/image_raw'),
			('rgb/camera_info', '/camera/color/camera_info'),
			('depth/image', '/camera/aligned_depth_to_color/image_raw')]

		# Map or Localization mode:
		ld.add_action(Node(
				package='rtabmap_slam', executable='rtabmap', output='screen',
				parameters=parameters,
				            #{'Mem/IncrementalMemory': 'False',
				            #'Mem/InitWMWithAllNodes': 'True'}],
				remappings=remappings
		))

		ld.add_action(Node(
				package='rtabmap_viz', executable='rtabmap_viz', output='screen',
				parameters=parameters,
				remappings=remappings
		))

		# Compute quaternion of the IMU
		ld.add_action(Node(
					package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
					parameters=[{'use_mag': False,
					             "world_frame": "camera_link",
					             "publish_tf": False
					             }],
					remappings=[('/imu/data_raw', '/camera/imu')
					            #("/imu/data", "/rtabmap/imu")
					            ]
		))

		"""ld.add_action(Node(
				package='rtabmap_ros', executable='rgbd_odometry', output='screen',
				parameters=parameters,
				remappings=remappings))

		ld.add_action(Node(
				package='rtabmap_ros', executable='rtabmap', output='screen',
				parameters=parameters,
				remappings=remappings,
				arguments=['-d']))

		ld.add_action(Node(
				package='rtabmap_ros', executable='rtabmapviz', output='screen',
				parameters=parameters,
				remappings=remappings))

		# Because of this issue: https://github.com/IntelRealSense/realsense-ros/issues/2564
		# Generate point cloud from not aligned depth
		ld.add_action(Node(
					package='rtabmap_ros', executable='point_cloud_xyz', output='screen',
					parameters=[{'approx_sync': True}],
					remappings=remappings
		))


			# Compute quaternion of the IMU
		ld.add_action(Node(
					package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
					parameters=[{'use_mag': False,
					             "world_frame": "enu",
					             "publish_tf": False
					             }],
					remappings=[('/imu/data_raw', '/camera/imu'),
					            ("/imu/data", "/rtabmap/imu")]
		))"""

		"""
			# Generate aligned depth to color camera from the point cloud above
		ld.add_action(Node(
					package='rtabmap_ros', executable='pointcloud_to_depthimage', output='screen',
					parameters=[{'decimation': 2,
					             'fixed_frame_id': 'camera_link',
					             'fill_holes_size': 1}],
					remappings=[('camera_info', '/camera/color/camera_info'),
					            ('cloud', '/camera/cloud_from_depth'),
					            ('image_raw', '/camera/realigned_depth_to_color/image_raw')]))

		))

			# The IMU frame is mising in TF tree, add it:
		ld.add_action(Node(
					package='tf2_ros', executable='static_transform_publisher', output='screen',
					arguments=['0', '0', '0', '0', '0', '0', 'camera_gyro_optical_frame', 'camera_imu_optical_frame']))"""

	# spot_description
	if 0:
		spot_launch = os.path.join(get_package_share_directory('spot_description'), 'launch', "description.launch.py")
		ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(spot_launch)))

	# Realsense
	if 1:
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