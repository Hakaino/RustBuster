# Import necessary ROS2 packages
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def setEnvVars():
	# Set environment variable
	os.environ["TURTLEBOT3_MODEL"] = "waffle"
	os.environ["GAZEBO_MODEL_PATH"] = "/opt/ros/humble/share/turtlebot3_gazebo/models"


# if not os.environ["TURTLEBOT3_MODEL"] or not os.environ["GAZEBO_MODEL_PATH"]:


def generate_launch_description():
	setEnvVars()

	# Get configuration files
	rviz_config = os.path.join(get_package_share_directory('rustbuster'), "rviz2_config.rviz")
	exploration_params_file = os.path.join(get_package_share_directory('rustbuster'), 'config/exploration_params.yaml')
	map_file = os.path.join(get_package_share_directory('rustbuster'), 'maps/map.yaml')

	# Define ROS2 parameters
	use_sim_time = LaunchConfiguration('use_sim_time', default='True')
	headless = LaunchConfiguration('headless', default="true")
	namespace = LaunchConfiguration('namespace', default='false')
	use_robot_state_pub = LaunchConfiguration('use_robot_state_pub', default='True')

	# Get launch files
	tb3_gazebo_launch_file = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch',
	                                      "turtlebot3_world.launch.py")
	nav2_launch_nav = os.path.join(get_package_share_directory("nav2_bringup"), 'launch', "navigation_launch.py")
	slam_launch_file = os.path.join(get_package_share_directory("slam_toolbox"), 'launch', "online_sync_launch.py")
	explore_launch_file = os.path.join(get_package_share_directory("explore_lite"), 'launch', "explore.launch.py")

	# Create launch description
	ld = LaunchDescription()
	ld.add_action(DeclareLaunchArgument(
			'use_sim_time',
			default_value=use_sim_time,
			description='RustBuster main launcher'
	))

	# Nav2
	if 1:
		ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(nav2_launch_nav)))

	# Rviz
	if 1:
		ld.add_action(Node(
				package='rviz2',
				namespace='rviz2',
				executable='rviz2',
				name='rviz2',
				arguments=[rviz_config],
		))

	# slam toolbox
	if 1:
		ld.add_action(IncludeLaunchDescription(
				PythonLaunchDescriptionSource(slam_launch_file),
				launch_arguments={
					'use_sim_time': use_sim_time,
					'publish_tf': 'true',
					'params_file': exploration_params_file,
				}.items()
		))

	# Launch the Turtlebot3 Gazebo simulation
	if 1:
		ld.add_action(IncludeLaunchDescription(
				PythonLaunchDescriptionSource(tb3_gazebo_launch_file),
				launch_arguments={
					'use_sim_time': use_sim_time,
					"headless": headless
				}.items()
		))

	# Robot state publisher
	if 1:
		# urdf = os.path.join(get_package_share_directory('nav2_bringup'), 'urdf', 'turtlebot3_waffle.urdf')
		urdf = os.path.join(get_package_share_directory('nav2_bringup'), 'urdf', 'turtlebot3_waffle.urdf')
		with open(urdf, 'r') as infp:
			robot_description = infp.read()

		ld.add_action(Node(
				condition=IfCondition(use_robot_state_pub),
				package='robot_state_publisher',
				executable='robot_state_publisher',
				name='robot_state_publisher',
				namespace=namespace,
				output='screen',
				parameters=[{'use_sim_time': use_sim_time,
				             'robot_description': robot_description}],
				remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
		))

	# Explorer
	if 1:
		ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(explore_launch_file)))


	# My controller
	if 0:
		ld.add_action(Node(
				package="rustbuster",
				executable="rustbuster_init",
				name='rustbuster_main',
				output='screen'
		))

	return ld
