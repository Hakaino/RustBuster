import rclpy
import tf2_ros
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu, PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


class RustBusterMain(Node):

	def __init__(self):
		super().__init__('rustbuster_main')
		# Subscribers
		self.switch = self.create_subscription(Bool, "rustbuster/explore", self.control, 1)
		self.odometry = self.create_subscription(Odometry, 'odometry', self.fake_imu, rclpy.qos.qos_profile_sensor_data)
		self.back_points = self.create_subscription(PointCloud2, 'points_back', self.pub_imu,
		                                            rclpy.qos.qos_profile_sensor_data)

		# Publishers
		self.explore = self.create_publisher(Bool, "explore/resume", 1)
		self.imu = self.create_publisher(Imu, "imu", 1)
		self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)

		self.imu_t = TransformStamped()
		self.imu_t.header.stamp = self.get_clock().now().to_msg()
		self.imu_t.header.frame_id = "head"
		self.imu_t._child_frame_id = "imu"
		self.imu_t.transform.rotation.w = -1.0
		self.broadcaster.sendTransform(self.imu_t)

		self.imu_msg = None
		self.get_logger().info("starting................................")

	def pub_imu(self, pcd2):
		self.imu.publish(self.imu_msg)

	def fake_imu(self, odom):
		# imu publish
		self.imu_msg = Imu()
		self.imu_msg.header = odom.header
		self.imu_msg.header.frame_id = "imu"
		self.imu_msg.orientation = odom.pose.pose.orientation
		self.imu_msg.angular_velocity = odom.twist.twist.angular
		self.imu_msg.linear_acceleration = odom.twist.twist.linear

	# self.get_logger().info("X")

	def control(self, auto):
		self.explore.publish(auto)
		msg = "auto"

		if not bool(auto.data):
			# this makes maps subprocess.run(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", "my_map",])
			msg = "manual"
		# subprocess.run(["ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard"])
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
