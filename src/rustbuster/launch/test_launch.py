# Import necessary ROS2 packages
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


# Define launch description
def generate_launch_description():
	# Set environment variable
	os.environ["TURTLEBOT3_MODEL"] = "waffle"
	os.environ["GAZEBO_MODEL_PATH"] = "/opt/ros/humble/share/turtlebot3_gazebo/models"
	#print(os.environ["TURTLEBOT3_MODEL"], "---------------------------------")
	#print(os.environ["GAZEBO_MODEL_PATH"], "---------------------------------")
	#if not os.environ["TURTLEBOT3_MODEL"] or not os.environ["GAZEBO_MODEL_PATH"]:
	#	print("TURTLEBOT3_MODEL is missing -----------------------")


	# Get configuration files
	rviz_config = os.path.join(get_package_share_directory('rustbuster'), "rviz2_config.rviz")
	exploration_params_file = os.path.join(get_package_share_directory('rustbuster'), 'config/exploration_params.yaml')
	map_file = os.path.join(get_package_share_directory('rustbuster'), 'maps/map.yaml')

	# Define ROS2 parameters
	use_sim_time = LaunchConfiguration('use_sim_time', default='True')
	headless = LaunchConfiguration('headless', default=True)
	slam = LaunchConfiguration('slam', default="True")
	namespace = LaunchConfiguration('namespace', default='false')
	use_namespace = LaunchConfiguration('use_namespace', default='false')
	map_yaml_file = LaunchConfiguration('map', default=map_file)
	params_file = LaunchConfiguration('params_file', default=exploration_params_file)
	autostart = LaunchConfiguration('autostart', default="true")
	use_composition = LaunchConfiguration('use_composition', default='True')
	use_respawn = LaunchConfiguration('use_respawn', default='False')





	# Launch configuration variables specific to simulation
	rviz_config_file = LaunchConfiguration('rviz_config_file')
	use_simulator = LaunchConfiguration('use_simulator', default='True')
	use_robot_state_pub = LaunchConfiguration('use_robot_state_pub', default='True')
	world = LaunchConfiguration('world')
	pose = {'x': LaunchConfiguration('x_pose', default='-2.00'),
	        'y': LaunchConfiguration('y_pose', default='-0.50'),
	        'z': LaunchConfiguration('z_pose', default='0.01'),
	        'R': LaunchConfiguration('roll', default='0.00'),
	        'P': LaunchConfiguration('pitch', default='0.00'),
	        'Y': LaunchConfiguration('yaw', default='0.00')}
	robot_name = LaunchConfiguration('robot_name')
	robot_sdf = LaunchConfiguration('robot_sdf')


	# Get launch files
	tb3_gazebo_launch_file = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch',
	                                      "turtlebot3_world.launch.py")
	# nav2_launch_file = os.path.join(get_package_share_directory("rustbuster"), 'launch', "nav2_bringup_launch.py")
	nav2_launch_file = os.path.join(get_package_share_directory("nav2_bringup"), 'launch', "navigation_launch.py")
	slam_launch_file = os.path.join(get_package_share_directory("slam_toolbox"), 'launch', "online_sync_launch.py")


	# Create launch description
	ld = LaunchDescription()

	# Declare ROS2 parameters
	ld.add_action(DeclareLaunchArgument(
			'use_sim_time',
			default_value=use_sim_time,
			description='Use simulation (Gazebo) clock if true'
	))

	# Rviz
	if True:
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
				#launch_arguments={"headless": headless}.items()
				launch_arguments={

					"headless": headless,
					"namespace": namespace,
	                "use_namespace": use_namespace,
	                "slam": slam,
	                "map": map_yaml_file,
	                "use_sim_time": use_sim_time,
	                "params_file": params_file,
	                "autostart": autostart,
	                "use_composition": use_composition,
	                "use_respawn": use_respawn
	                }.items()
		))

	# slam toolbox
	if True:
		ld.add_action(IncludeLaunchDescription(
				PythonLaunchDescriptionSource(slam_launch_file),
				launch_arguments={
					'use_sim_time': use_sim_time,
					'publish_tf': 'true',
					'params_file': exploration_params_file,
				}.items()
		))

	# Launch the Turtlebot3 Gazebo simulation
	if True:
		ld.add_action(IncludeLaunchDescription(
				PythonLaunchDescriptionSource(tb3_gazebo_launch_file),
				launch_arguments={'use_sim_time': use_sim_time}.items()
		))

	# Robot state publisher
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
