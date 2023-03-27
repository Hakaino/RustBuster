# Import necessary ROS2 packages
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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
	#ld.add_action((src='points_back', dst='horizontal_laser_3d'))
	#ld.add_action(SetRemap(src='points_frontleft', dst='vertical_laser_3d'))

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
				output='screen'
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

	# Launch spot driver
	if 1:
		spot_config = os.path.join(get_package_share_directory('rustbuster'), 'config/spot_config.yaml')
		spot_launch = os.path.join(get_package_share_directory('spot_driver'), 'launch', "spot_driver.launch.py")
		ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(spot_launch),
		                                       launch_arguments={"config_file": spot_config}.items()))

	# point_cloud_xyz
	if 1:
		depth_to_launch = os.path.join(get_package_share_directory("spot_driver"), 'launch',
		                               "point_cloud_xyz.launch.py")
		ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(depth_to_launch)))

	# cartographer
	if 1:
		cartographer_config = os.path.join(get_package_share_directory('rustbuster'), 'config/backpack_3d.lua')
		#cartographer_launch = os.path.join(get_package_share_directory("cartographer_ros"), 'launch', "backpack_3d.launch.py")
		#ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(cartographer_launch)))


		## ***** Nodes *****
		ld.add_action(Node(
				package='cartographer_ros',
				executable='cartographer_node',
				#parameters=[{'use_sim_time': "False"}],
				arguments=[
					'-configuration_directory',
					get_package_share_directory('rustbuster') + '/config',
					'-configuration_basename', "backpack_3d.lua"],
				remappings=[
					('points2_1', 'horizontal_laser_3d'),
					('points2_2', 'vertical_laser_3d')],
				output='screen'
		))

		ld.add_action(Node(
				package='cartographer_ros',
				executable='cartographer_occupancy_grid_node',
				parameters=[
					#{'use_sim_time': "False"},
					{'resolution': 0.05}],
		))

	# Explorer
	if 0:
		explore_launch = os.path.join(get_package_share_directory("explore_lite"), 'launch', "explore.launch.py")
		ld.add_action(IncludeLaunchDescription(
				PythonLaunchDescriptionSource(explore_launch),
				launch_arguments={
					'use_sim_time': "False",
					"headless": "True"
				}.items()
		))

	# going another way

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
		ld.add_action(IncludeLaunchDescription(
				PythonLaunchDescriptionSource(slam_launch),
				launch_arguments={
					'use_sim_time': "False",
					'publish_tf': "True",
					'params_file': exploration_params_file,
				}.items(),
		))

	# Ouster
	if 0:
		ouster_launch = os.path.join(get_package_share_directory("ros2_ouster"), 'launch', "driver_launch.py")
		ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(ouster_launch)))

	return ld
