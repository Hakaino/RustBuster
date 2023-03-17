# Import necessary ROS2 packages
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setEnvVars():
	# Set environment variable
	os.environ["TURTLEBOT3_MODEL"] = "waffle"
	os.environ["GAZEBO_MODEL_PATH"] = "/opt/ros/humble/share/turtlebot3_gazebo/models"




def generate_launch_description():
	setEnvVars()

	# Get configuration files
	rviz_config = os.path.join(get_package_share_directory('rustbuster'), "config/spot_rviz2_config.rviz")
	cartographer_config = os.path.join(get_package_share_directory('rustbuster'), "config/backpack_3d.lua")
	#rviz_config = os.path.join(get_package_share_directory("rustbuster"), "config/sim_rviz2_config.rviz")
	exploration_params_file = os.path.join(get_package_share_directory('rustbuster'), "config/exploration_params.yaml")
	map_file = os.path.join(get_package_share_directory('rustbuster'), "maps/map.yaml")

	# Get launch files
	tb3_gazebo_launch_file = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', "turtlebot3_world.launch.py")
	nav2_launch = os.path.join(get_package_share_directory("nav2_bringup"), 'launch', "navigation_launch.py")
	cartographer_launch = os.path.join(get_package_share_directory("cartographer_ros"), 'launch', "backpack_3d.launch.py")
	slam_launch = os.path.join(get_package_share_directory("slam_toolbox"), 'launch', "online_sync_launch.py") # "lifelong_launch.py")
	explore_launch = os.path.join(get_package_share_directory("explore_lite"), 'launch', "explore.launch.py")
	#kimera_launch_launch = os.path.join(get_package_share_directory(""), 'launch', "")

	# Create launch description
	ld = LaunchDescription()
	ld.add_action(DeclareLaunchArgument(
			'use_sim_time',
			default_value="False",
			description='RustBuster main launcher'
	))

	# Nav2
	if 0:
		ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(nav2_launch)))

	# Rviz
	if 1:
		ld.add_action(Node(
				package='rviz2',
				namespace='rviz2',
				executable='rviz2',
				name='rviz2',
				arguments=["-d", rviz_config],
		))

	# cartogrpher
	if 1:
		ld.add_action(IncludeLaunchDescription(
				PythonLaunchDescriptionSource(cartographer_launch),
				launch_arguments={'use_sim_time': "True"}.items()
		))
				#remappings=[
				#	('points2_1', 'horizontal_laser_3d'),
				#	('points2_2', 'vertical_laser_3d')]

	# slam toolbox
	if 0:
		ld.add_action(IncludeLaunchDescription(
				PythonLaunchDescriptionSource(slam_launch),
				launch_arguments={
					'use_sim_time': "False",
					'publish_tf': 'True',
					'params_file': exploration_params_file,
				}.items()
		))

	# Kimera
	if 0:
		ld.add_action(IncludeLaunchDescription(
				PythonLaunchDescriptionSource(kimera_launch),
				# launch_arguments={
				# 	'use_sim_time': use_sim_time,
				# 	'publish_tf': 'true',
				# 	'params_file': exploration_params_file,
				# }.items()
		))


	# Launch the Turtlebot3 Gazebo simulation
	if 1:
		ld.add_action(IncludeLaunchDescription(
				PythonLaunchDescriptionSource(tb3_gazebo_launch_file),
				launch_arguments={
					'use_sim_time': "False",
					"headless": "True"
				}.items()
		))

	# Explorer
	if 0:
		ld.add_action(IncludeLaunchDescription(
				PythonLaunchDescriptionSource(explore_launch),
				launch_arguments={
					'use_sim_time': use_sim_time,
					"headless": headless
				}.items()
		))


	# My controller
	if 0:
		ld.add_action(Node(
				package="rustbuster",
				executable="rustbuster_init",
				name='rustbuster_main',
				output='screen'
		))

	return ld
