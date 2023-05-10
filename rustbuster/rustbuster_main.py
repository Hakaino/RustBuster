import rclpy
import tf2_ros
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu, PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Vector3


class RustBusterMain(Node):

	def __init__(self):
		super().__init__('rustbuster_main')
		# Subscribers
		self.switch = self.create_subscription(Bool, "rustbuster/explore", self.control, 1)
		self.odometry_sub = self.create_subscription(Odometry, "odometry", self.fake_odom, rclpy.qos.qos_profile_sensor_data)
		#self.imu_sub = self.create_subscription(Imu, "/imu/data", self.cov_imu, 1)

		# Publishers
		self.explore = self.create_publisher(Bool, "explore/resume", 1)
		#self.imu_pub = self.create_publisher(Imu, "imu", 10)
		self.odom_pub = self.create_publisher(Odometry, "/odom", 1)
		self.broadcaster = tf2_ros.TransformBroadcaster(self)

		e = Bool()
		e.data = False
		self.explore.publish(e) # start exploration here
		self.get_logger().info("starting................................")

	def fake_imu(self, odom):
		#self.get_logger().info("ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ")
		# change odometry
		t = TransformStamped()
		t.header.stamp = odom.header.stamp
		t.header.frame_id = "odom"
		t.child_frame_id = "base_link"
		t.transform.translation.x = odom.pose.pose.position.x
		t.transform.translation.y = odom.pose.pose.position.y
		t.transform.translation.z = odom.pose.pose.position.z
		t.transform.rotation = odom.pose.pose.orientation
		self.broadcaster.sendTransform(t)

		# imu publish
		imu_msg = Imu()
		imu_msg.header = odom.header
		imu_msg.header.frame_id = "imu"
		imu_msg.orientation = odom.pose.pose.orientation
		imu_msg.angular_velocity = odom.twist.twist.angular
		#imu_msg.angular_velocity.x = 0.1
		imu_msg.linear_acceleration = odom.twist.twist.linear
		#imu_msg.linear_acceleration.z = -10.0
		self.imu_pub.publish(imu_msg)

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

	def cov_imu(self, imu):
		new_imu = imu
		new_imu.orientation_covariance = imu.linear_acceleration_covariance
		self.imu_pub.publish(new_imu)

	def fake_odom(self, old_odom):
		odom = Odometry()
		odom.header.stamp = self.get_clock().now().to_msg()
		odom.header.frame_id = "odom"
		odom.child_frame_id = "camera_link"
		odom.pose = old_odom.pose
		odom.twist = old_odom.twist
		self.odom_pub.publish(odom)


def main(args=None):
	rclpy.init(args=args)
	rustbuster_main = RustBusterMain()
	rclpy.spin(rustbuster_main)
	rustbuster_main.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
