# Import necessary ROS2 packages
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, actions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
	# Get configuration files
	exploration_params_file = os.path.join(get_package_share_directory('rustbuster'), 'config/exploration_params.yaml')
	map_file = os.path.join(get_package_share_directory('rustbuster'), 'maps/map.yaml')

	# Create launch description
	ld = LaunchDescription()
	ld.add_action(DeclareLaunchArgument('use_sim_time', default_value="False", description='RustBuster main launcher'))

	# Ros2 bag
	if 1:
		bag_path = "rosbag2_2023_04_12-14_17_02"
		ld.add_action(actions.ExecuteProcess(
				cmd=['ros2', 'bag', 'play', bag_path, "--topics", "/odometry", "/points_back", "/points_left",
				     "/points_right", "/points_frontleft", "/points_frontright", "/camera/back/image",
				     "/robot_description", "/tf", "/tf_static"],
				output='log'))  # ld.add_action(actions.ExecuteProcess( cmd=['ros2', 'bag', 'record', "--all"], output='log' ))

	# Nav2
	if 0:
		nav2_launch = os.path.join(get_package_share_directory("nav2_bringup"), 'launch', "navigation_launch.py")
		ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(nav2_launch)))

	# My controller
	if 1:
		ld.add_action(Node(package="rustbuster", executable="rustbuster_init", name='rustbuster_main', output='screen'))

	# RVIZ2
	if 1:
		ld.add_action(Node(package='rviz2', namespace='rviz2', executable='rviz2', name='rviz2', arguments=["-d",
		                                                                                                    os.path.join(
			                                                                                                    get_package_share_directory(
				                                                                                                    "rustbuster"),
			                                                                                                    "config",
			                                                                                                    "spot_rviz2_config.rviz")],
				output='log'))

	# Launch spot driver
	if 0:
		spot_config = os.path.join(get_package_share_directory('rustbuster'), 'config/spot_config.yaml')
		spot_launch = os.path.join(get_package_share_directory('spot_driver'), 'launch', "spot_driver.launch.py")
		ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(spot_launch),
		                                       launch_arguments={"config_file": spot_config}.items()))

	# point_cloud_xyz
	if 0:
		depth_to_launch = os.path.join(get_package_share_directory("spot_driver"), 'launch',
		                               "point_cloud_xyz.launch.py")
		ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(depth_to_launch)))

	# Point cloud to laser scan
	if 0:
		ld.add_action(Node(package='pointcloud_to_laserscan', executable='laserscan_to_pointcloud_node',
				name='laserscan_to_pointcloud',
				remappings=[('scan_in', [LaunchConfiguration(variable_name='scanner'), '/horizontal_laser_3d']),
				            ('cloud', [LaunchConfiguration(variable_name='scanner'), '/points_frontright'])],
				parameters=[{'target_frame': 'frontright', 'transform_tolerance': 0.01}]))

	# cartographer
	if 1:
		# cartographer_config = os.path.join(get_package_share_directory('rustbuster'), 'config/backpack_3d.lua')
		# cartographer_launch = os.path.join(get_package_share_directory("cartographer_ros"), 'launch', "backpack_3d.launch.py")
		# ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(cartographer_launch)))

		## ***** Nodes *****
		ld.add_action(
				Node(package='cartographer_ros', executable='cartographer_node', parameters=[{'use_sim_time': False}],
						arguments=['-configuration_directory', get_package_share_directory('rustbuster') + '/config',
							'-configuration_basename', "backpack_3d.lua"],
						remappings=[('points2_1', 'points_frontright'), ('points2_2', 'points_frontleft'),
							('points2_3', 'points_right'), ('points2_4', 'points_left'), ('points2_5', 'points_back'),
							('odom', 'odometry')], output='screen'))

		ld.add_action(Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
				parameters=[{'use_sim_time': False}, {'resolution': 0.05}], ))

	# Explorer
	if 0:
		explore_launch = os.path.join(get_package_share_directory("explore_lite"), 'launch', "explore.launch.py")
		ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(explore_launch),
				launch_arguments={'use_sim_time': "False", "headless": "True"}.items()))

	"""going another way"""

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
		slam_launch = os.path.join(get_package_share_directory("slam_toolbox"), 'launch', "online_sync_launch.py")
		ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(slam_launch),
				launch_arguments={'use_sim_time': "False", 'publish_tf': "True",
					'params_file': exploration_params_file, }.items(), ))

	# Ouster
	if 0:
		ouster_launch = os.path.join(get_package_share_directory("ros2_ouster"), 'launch', "driver_launch.py")
		ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(ouster_launch)))

	return ld
