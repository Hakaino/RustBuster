# Import necessary ROS2 packages
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, actions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import launch_ros


def generate_launch_description():
	ld = LaunchDescription()
	ld.add_action(DeclareLaunchArgument('use_sim_time', default_value = "False", description='to change when using simulations or bag files'))
	#ld.add_action(Node(package='tf2_ros', executable='static_transform_publisher',
	#                   arguments=['.0', '.0', '.0', '.0', '.0', '.0', 'map', 'odom']))

	ld.add_action(Node(package='tf2_ros', executable='static_transform_publisher',
	                   arguments=['.0', '.0', '.0', '.0', '.0', '.0', 'base_link', 'body']))
	ld.add_action(Node(package='tf2_ros', executable='static_transform_publisher',
	                   arguments=['.0', '.0', '.0', '.0', '.0', '.0', 'base_link', 'body']))
	ld.add_action(Node(package='tf2_ros', executable='static_transform_publisher',
	                   arguments=['.0', '.03', '.0', '.0', '.0', '.0', 'base_link', 'camera_link'])) # above front_rail would be more correct
	"""ld.add_action(Node(package='tf2_ros', executable='static_transform_publisher',
	                   arguments=['.0', '.0', '.0', '.0', '.0', '.0', 'camera_link', "imu"]))
	ld.add_action(Node(package='tf2_ros', executable='static_transform_publisher',
	                   arguments=['.0', '.0', '.0', '.0', '.0', '.0', 'camera_gyro_optical_frame', 'camera_imu_optical_frame']))"""


	# Ros2 bag
	if 0:
		#bag_path = "rosbag2_2023_04_26-15_36_29"
		#ld.add_action(actions.ExecuteProcess(cmd=['ros2', 'bag', 'play', bag_path]))
		"""			, "--topics", "/odometry", "/points_back", "/points_left", "/points_right", "/points_frontleft"
					, "/points_frontright", "/camera/back/image", "/robot_description", "/tf", "/tf_static"],
					, "/depth/frontleft/image", "/camera/frontleft/camera_info" ],
					 output='log'))"""

		ld.add_action(actions.ExecuteProcess( cmd=['ros2', 'bag', 'record', "--all"], output='log' ))

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
	if 1:
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

	# Explorer
	if 0:
		ld.add_action(Node(
				package="explore_lite",
				name="explore_node",
				executable="explore",
				parameters=[os.path.join(get_package_share_directory("rustbuster"), "config", "front_expl_param.yaml")]
		))

	# rtabmaps (commented out parts to find the bug in the installation)
	if 1:
		parameters = [{
			'frame_id':'base_link',
			'visual_odometry': False,
			'odom_frame_id': "odom",
			'map_always_update':False,
			'wait_for_transform':0.5,
			#'subscribe_rgbd': True,
			#'subscribe_depth':True,
			'approx_sync':True,
			'wait_imu_to_init':True,
            #'subscribe_rgb':True,
            #'subscribe_scan':False,
			#'use_action_for_goal':True,
			'cloud_noise_filtering_radius':0.05,
			'cloud_noise_filtering_min_neighbors':2,

			'proj_max_ground_angle':45,
			'proj_min_cluster_size':20,
			'proj_max_ground_height':0.2,

            #'qos_scan':2,
	        #'qos_imu':2,
			"rtabmap_args": "-d " + os.path.join(get_package_share_directory('rustbuster'), 'config/rtabmap.ini')
		}]

		remappings = [
			('imu', '/imu/data'),
			('rgb/image', '/camera/color/image_raw'),  # "/camera/infra1/image_rect_raw"),  #
			('rgb/camera_info', '/camera/color/camera_info'), # "/camera/infra1/camera_info"),  #
			('depth/image', '/camera/aligned_depth_to_color/image_raw'),
			('depth/image_info', '/camera/aligned_depth_to_color/image_raw_info')
		]

		# Map or Localization mode:
		ld.add_action(Node(
				package='rtabmap_slam', executable='rtabmap', output='screen',
				parameters=parameters,
				            #{'Mem/IncrementalMemory': 'False',
				            #'Mem/InitWMWithAllNodes': 'True'}],
				remappings=remappings
		))

		"""ld.add_action(Node(
				package='rtabmap_viz', executable='rtabmap_viz', output='log',
				parameters=parameters,
				remappings=remappings
		))"""

		# Compute quaternion of the IMU
		ld.add_action(Node(
					package='imu_filter_madgwick', executable='imu_filter_madgwick_node', #output='screen',
					parameters=[{'use_mag': False,
					             "world_frame": "odom",
					             "publish_tf": False
					             }],
					remappings=[('/imu/data_raw', '/camera/imu')]
		))

	# spot_description
	if 0:
		default_model_path = os.path.join(get_package_share_directory('spot_description'), 'urdf/spot.urdf.xacro')
		ld.add_action(Node(
				package='robot_state_publisher',
				executable='robot_state_publisher',
				parameters=[{'robot_description': Command(['xacro ', default_model_path])}]
		))

	# Realsense
	if 1:
		realsense_launch = os.path.join(get_package_share_directory("rustbuster"), 'launch', "rs_launch.py")
		ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(realsense_launch)))

	# Apriltags
	if 0:
		ld.add_action(Node(
				package="apriltag_ros",
				executable='apriltag_node',
				remappings=[
					('image_rect', '/camera/color/image_raw'),
					('camera_info', '/camera/color/camera_info'),
				],
				parameters=[{
					"tag_family": "tag36h11",  # tag36h10, tag25h9, tag25h7 and tag16h5
					"tag_border": 1,
					"tag_threads": 4,
					"tag_decimate": 1.0,
					"tag_blur": 0.0,
					"tag_refine_edges": 1,
					"tag_refine_decode": 0,
					"tag_refine_pose": 0,
					"publish_tf": True,
					"camera_frame": "camera_link",
					"publish_tag_detections": True
				}],
		))

	return ld