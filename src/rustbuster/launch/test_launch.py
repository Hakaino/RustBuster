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
	# Set the TURTLEBOT3_MODEL environment variable
	tb3_model = os.environ["TURTLEBOT3_MODEL"] if "TURTLEBOT3_MODEL" in os.environ else 'waffle'
	set_env_var = SetEnvironmentVariable("TURTLEBOT3_MODEL", tb3_model)
	#gazebo_model = os.environ["GAZEBO_MODEL_PATH"] if "GAZEBO_MODEL_PATH" in os.environ else "/opt/ros/humble/share" \
	#                                                                                         "/turtlebot3_gazebo/models"
	#set_env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model)

	# Define ROS2 parameters
	use_sim_time = LaunchConfiguration('use_sim_time', default='false')

	# Get the path to the Turtlebot3 Gazebo launch file
	tb3_gazebo_launch_file = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch',
	                                      'turtlebot3_world.launch.py')
	# Get configuration files
	rviz_config = os.path.join(get_package_share_directory('rustbuster'), "rviz2_config.rviz")
	exploration_params_file = os.path.join(get_package_share_directory('rustbuster'), 'exploration_params.yaml')

	# Create launch description
	ld = LaunchDescription()

	# Rviz
	if 1:
		ld.add_action(Node(
					package='rviz2',
					namespace='rviz2',
					executable='rviz2',
					name='rviz2',
					arguments=[rviz_config],
		))

	# Declare ROS2 parameters
	# ld.add_action(DeclareLaunchArgument(
	# 		'use_sim_time',
	# 		default_value=use_sim_time,
	# 		description='Use simulation (Gazebo) clock if true'
	# ))
	print("rviz-----------------")

	# # Launch the Turtlebot3 Gazebo simulation
	# ld.add_action(IncludeLaunchDescription(
	# 		PythonLaunchDescriptionSource(tb3_gazebo_launch_file),
	# 		launch_arguments={'use_sim_time': use_sim_time}.items()
	# ))
	print("rviz-----------------")

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
