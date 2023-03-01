import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import Pose


class RustBusterMain(Node):

	def __init__(self):
		super().__init__('rustbuster_main')
		self.get_logger().info("OOOOOOOOOO----------------")
		self.nav = BasicNavigator()
		#self.nav.waitUntilNav2Active()  # if autostarted, else use `lifecycleStartup()`
		self.goal = self.create_subscription(Pose, "goal_pose", self.goTo, 10)
		self.initialPose = self.create_subscription(Pose, "pose", self.setPose, 10)
		self.initialPose = self.create_subscription(Pose, "map", self.mapListener, 10)
		self.get_logger().info("11111111----------------")


	def mapListener(self, map):
		self.get_logger().info("map change----------------")


	def setPose(self, init_pose):
		self.get_logger().info("definning current pose----------------")
		self.nav.setInitialPose(init_pose)

	def goTo(self, goal_pose):
		self.get_logger().info("go to goal----------------")
		path = self.nav.getPath(self.init_pose, goal_pose)
		smoothed_path = self.nav.smoothPath(path)
		self.nav.goToPose(goal_pose)
		while not self.nav.isTaskComplete():
			feedback = self.nav.getFeedback()
			if feedback.navigation_duration > 600:
				self.nav.cancelTask()

		"""result = nav.getResult()
		if result == TaskResult.SUCCEEDED:
			print('Goal succeeded!')
		elif result == TaskResult.CANCELED:
			print('Goal was canceled!')
		elif result == TaskResult.FAILED:
			print('Goal failed!')"""


def main(args=None):
	rclpy.init(args=args)
	rustbuster_main = RustBusterMain()
	rclpy.spin(rustbuster_main)
	rustbuster_main.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
