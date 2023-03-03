import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from launch_ros.actions import Node as nd
import subprocess


class RustBusterMain(Node):

	def __init__(self):
		super().__init__('rustbuster_main')
		# self.nav = BasicNavigator()
		# self.nav.waitUntilNav2Active()  # if autostarted, else use `lifecycleStartup()`
		# self.goal = self.create_subscription(Pose, "goal_pose", self.goTo, 10)
		self.switch = self.create_subscription(Bool, "rustbuster/explore", self.control, 10)
		self.explore = self.create_publisher(Bool, "explore/resume", 10)
		# self.teleop_node = nd("teleop_twist_keyboard", "teleop_twist_keyboard")
		# self.launch = launch_ros.scriptapi.ROSLaunch()
		# self.launch.start()
		# self.teleop = self.launch.launch(self.teleop_node)

	def readMessage(self, message):
		msg = str(message.data)
		self.get_logger().info(msg)

	def control(self, auto):
		self.explore.publish(bool(auto.data))
		msg = "auto"

		if not bool(auto.data):
			# this makes maps subprocess.run(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", "my_map",])
			msg = "manual"
			#subprocess.run(["ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard"])
			# self.teleop = self.launch.launch(self.teleop_node)
			# self.get_logger().info("change: ", self.teleop.is_alive())
			# self.teleop.stop()
			# start teleop
		self.get_logger().info("Change to " + msg + " mode")


def main(args=None):
	rclpy.init(args=args)
	rustbuster_main = RustBusterMain()
	rclpy.spin(rustbuster_main)
	rustbuster_main.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
