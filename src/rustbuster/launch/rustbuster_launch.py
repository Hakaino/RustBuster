import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	name = "turtlebot"  # "spot"

	l = 50
	m = 3 * " " + name + 3 * " "
	print("\n\n" + l * "#" + "\n" + int((l - len(m)) / 2) * "#"
	      + m +
	      int((l - len(m)) / 2) * "#" + "\n" + l * "#" + "\n\n")

	# Robot description
	urdf_file_name = "name" + ".urdf.xacro"
	pkg_share = FindPackageShare('rustbuster').find('rustbuster')
	#urdf_dir = os.path.join(pkg_share, 'urdf')
	#xacro_file = os.path.join(urdf_dir, name + ".urdf.xacro")
	xacro_file = os.path.join(pkg_share, "turtlebot_common_library.urdf.xacro")
	print(".........................")
	doc = xacro.process_file(xacro_file)
	robot_desc = doc.toprettyxml(indent='  ')



	use_sim_time = LaunchConfiguration('use_sim_time', default='false')

	# urdf = os.path.join(
	# 		get_package_share_directory('rustbuster'),
	# 		urdf_file_name)
	# with open(urdf, 'r') as infp:
	# 	robot_desc = infp.read()

	return LaunchDescription([
		DeclareLaunchArgument(
				'use_sim_time',
				default_value='false',
				description='Use simulation (Gazebo) clock if true'),
		Node(
				package='robot_state_publisher',
				executable='robot_state_publisher',
				name='robot_state_publisher',
				output='screen',
				parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
				arguments=[xacro_file]),
		# Node(
		#     package='urdf_tutorial_r2d2',
		#     executable='state_publisher',
		#     name='state_publisher',
		#     output='screen'),
	])
