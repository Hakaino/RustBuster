import rclpy
import tf2_ros
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu, PointCloud2, PointField
from nav_msgs.msg import Odometry
#from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped, Vector3
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException


class RustBusterMain(Node):

	def __init__(self):
		super().__init__('rustbuster_main')
		# Subscribers
		self.switch = self.create_subscription(Bool, "rustbuster/explore", self.control, 1)
		self.odometry_sub = self.create_subscription(Odometry, "odometry", self.fake_odom, 10)  #rclpy.qos.qos_profile_sensor_data)
		#self.detections_sub = self.create_subscription(AprilTagDetectionArray, "/detections", self.apriltag_detections, 1)
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)

		# Publishers
		self.explore = self.create_publisher(Bool, "explore/resume", 1)
		#self.imu_pub = self.create_publisher(Imu, "imu", 10)
		self.odom_pub = self.create_publisher(Odometry, "odom", 10)
		self.broadcaster = tf2_ros.TransformBroadcaster(self)

		e = Bool()
		e.data = True
		self.explore.publish(e) # start exploration here
		self.get_logger().info("starting................................")

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
		try:
			odom = Odometry()
			odom.header.stamp = old_odom.header.stamp  # self.get_clock().now().to_msg()
			odom.header.frame_id = "odom"
			odom.child_frame_id = "base_link"
			odom.pose = old_odom.pose
			odom.twist = old_odom.twist
			self.odom_pub.publish(odom)

			# use spots visual odometry transform
			t = self.tf_buffer.lookup_transform("vision", "base_link", rclpy.time.Time())
			#t.header.stamp = odom.header.stamp
			t.header.frame_id = "odom"
			t.child_frame_id = "base_link"
			self.broadcaster.sendTransform(t)
		except TransformException as ex:
			self.get_logger().info(f'Could not transform {"vision"} to {"base_link"}: {ex}')


	#def apriltag_detections(self, detection_array):
	#	detection_array

def main(args=None):
	rclpy.init(args=args)
	rustbuster_main = RustBusterMain()
	rclpy.spin(rustbuster_main)
	rustbuster_main.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
