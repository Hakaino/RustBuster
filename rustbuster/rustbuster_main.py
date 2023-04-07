import rclpy
import tf2_ros
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class RustBusterMain(Node):

	def __init__(self):
		super().__init__('rustbuster_main')
		# Subscribers
		self.switch = self.create_subscription(Bool, "rustbuster/explore", self.control, rclpy.qos.qos_profile_sensor_data)
		self.odometry = self.create_subscription(Odometry, 'odometry', self.fake_imu, rclpy.qos.qos_profile_sensor_data)
		self.back_points = self.create_subscription(PointCloud2, 'points_back', lambda pcd: self.merge_clouds(pcd, "back"), rclpy.qos.qos_profile_sensor_data)
		self.left_points = self.create_subscription(PointCloud2, 'points_left', lambda pcd: self.merge_clouds(pcd, "left"), rclpy.qos.qos_profile_sensor_data)
		self.right_points = self.create_subscription(PointCloud2, 'points_right', lambda pcd: self.merge_clouds(pcd, "right"), rclpy.qos.qos_profile_sensor_data)
		self.fright_points = self.create_subscription(PointCloud2, 'horizontal_laser_3d', lambda pcd: self.merge_clouds(pcd, "front_left"), rclpy.qos.qos_profile_sensor_data)
		self.fleft_points = self.create_subscription(PointCloud2, 'vertical_laser_3d', lambda pcd: self.merge_clouds(pcd, "front_right"), rclpy.qos.qos_profile_sensor_data)

		self.get_logger().info("still alive................................")

		# Publishers
		self.explore = self.create_publisher(Bool, "explore/resume", 1)
		self.imu = self.create_publisher(Imu, "imu", 10)
		self.odom_pub = self.create_publisher(Odometry, "odom", 10)
		self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
		self.combo_cloud = self.create_publisher(PointCloud2, "combo_cloud", 10)

		self.imu_t = TransformStamped()
		self.imu_t.header.stamp = self.get_clock().now().to_msg()
		self.imu_t.header.frame_id = "head"
		self.imu_t._child_frame_id = "imu"
		self.imu_t.transform.translation.x = 0.0
		self.imu_t.transform.translation.y = 0.0
		self.imu_t.transform.translation.z = 0.0
		self.imu_t.transform.rotation.x = 0.0
		self.imu_t.transform.rotation.y = 0.0
		self.imu_t.transform.rotation.z = 0.0
		self.imu_t.transform.rotation.w = 1.0
		self.broadcaster.sendTransform(self.imu_t)

		# Cloud buffer to combine everything
		self.cloud_buffer = {}


	# self.nav = BasicNavigator()
	# self.nav.waitUntilNav2Active()  # if autostarted, else use `lifecycleStartup()`
	# self.goal = self.create_subscription(Pose, "goal_pose", self.goTo, 10)

	# self.teleop_node = nd("teleop_twist_keyboard", "teleop_twist_keyboard")
	# self.launch = launch_ros.scriptapi.ROSLaunch()
	# self.launch.start()
	# self.teleop = self.launch.launch(self.teleop_node)

	# def readMessage(self, message):
	# msg = str(message.data)
	# self.get_logger().info(msg)
	def merge_clouds(self, pcd, camera):
		#self.get_logger().info("merging cloud................................")
		# Merge all point clouds into a buffer
		time_stamp = str(pcd.header.stamp.sec) # TODO: change to nano seconds
		if len(self.cloud_buffer.keys()) < 10 and not (time_stamp in self.cloud_buffer.keys()):
			self.cloud_buffer[time_stamp] = {"points": pcd}
		else:
			self.cloud_buffer[time_stamp]["points"].data = self.cloud_buffer[time_stamp]["points"].data + pcd.data
			#self.get_logger().info("adding points................................")
		cloud = self.cloud_buffer[time_stamp]
		cloud[camera] = True

		# Publish the combined point cloud if the buffer has data from all cameras
		try:
			if all([cloud[key] for key in ["back", "left", "right", "front_left", "front_right"]]):
				cloud["points"].is_dense = True
				cloud["points"].height = 1
				cloud["points"].width = len(cloud["points"].data)
				cloud["points"].point_step = 1
				self.combo_cloud.publish(self.cloud_buffer.pop(time_stamp)["points"])
				#self.get_logger().info("published combined cloud...................................")
		except KeyError:
			pass



	def fake_imu(self, odom):
		#self.get_logger().info("fake_imu")

		# tf message
		"""
		odom_t = TransformStamped()
		odom_t.header.stamp = odom.header.stamp
		odom_t.header.frame_id = "base_link"
		odom_t._child_frame_id = "imu"
		odom_t.transform.translation.x = odom.pose.pose.position.x
		odom_t.transform.translation.y = odom.pose.pose.position.y
		odom_t.transform.translation.z = odom.pose.pose.position.z
		odom_t.transform.rotation = odom.pose.pose.orientation
		self.broadcaster.sendTransform(odom_t)
		"""

		# imu publish
		imu_msg = Imu()
		imu_msg.header = odom.header
		imu_msg.header.frame_id = "imu"
		imu_msg.orientation = odom.pose.pose.orientation
		imu_msg.angular_velocity = odom.twist.twist.angular
		imu_msg.linear_acceleration = odom.twist.twist.linear
		self.imu.publish(imu_msg)
		self.odom_pub.publish(odom)

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
