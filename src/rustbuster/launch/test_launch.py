# Import necessary ROS2 packages
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Define launch description
def generate_launch_description():
	# Set environment variable
	os.environ.setdefault("TURTLEBOT3_MODEL", "waffle")
	os.environ.setdefault("GAZEBO_MODEL_PATH", "$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models")


	# Define ROS2 parameters
	use_sim_time = LaunchConfiguration('use_sim_time', default='false')
	headless = LaunchConfiguration('headless', default=False)

	# Get launch files
	tb3_gazebo_launch_file = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch',
	                                      "turtlebot3_world.launch.py")
	nav2_launch_file = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', "tb3_simulation_launch.py")

	# Get configuration files
	rviz_config = os.path.join(get_package_share_directory('rustbuster'), "rviz2_config.rviz")
	exploration_params_file = os.path.join(get_package_share_directory('rustbuster'), 'exploration_params.yaml')

	# Create launch description
	ld = LaunchDescription()

	# Rviz
	if False:
		ld.add_action(Node(
					package='rviz2',
					namespace='rviz2',
					executable='rviz2',
					name='rviz2',
					arguments=[rviz_config],
		))
	# Nav2
	if True:
		ld.add_action(IncludeLaunchDescription(
				PythonLaunchDescriptionSource(nav2_launch_file),
				launch_arguments={"headless": headless}.items()
		))


	# Declare ROS2 parameters
	# ld.add_action(DeclareLaunchArgument(
	# 		'use_sim_time',
	# 		default_value=use_sim_time,
	# 		description='Use simulation (Gazebo) clock if true'
	# ))

	# # Launch the Turtlebot3 Gazebo simulation
	# ld.add_action(IncludeLaunchDescription(
	# 		PythonLaunchDescriptionSource(tb3_gazebo_launch_file),
	# 		launch_arguments={'use_sim_time': use_sim_time}.items()
	# ))

	# Start exploration using the SLAM mapping node
	# ld.add_action(Node(
	# 		package='slam_toolbox',
	# 		executable='sync_slam_toolbox_node',
	# 		name='exploration',
	# 		output='screen',
	# 		emulate_tty=True,
	# 		arguments=['--ros-args', '--params-file', exploration_params_file, 'explore'],
	# 		parameters=[
	# 			{'use_sim_time': use_sim_time},
	# 			{'publish_tf': 'true'}
	# 		]
	# ))

	return ld
