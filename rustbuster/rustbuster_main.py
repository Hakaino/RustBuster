import rclpy
import tf2_ros
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu, PointCloud2, PointField
from nav_msgs.msg import Odometry
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped, Vector3
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
#from tf2_ros import quaternion_from_euler, quaternion_multiply
import math
import numpy as np
from example_interfaces.srv import AddTwoInts
from std_srvs.srv import Trigger, SetBool
from spot_msgs.msg import LeaseArray, LeaseResource
from rclpy.callback_groups import ReentrantCallbackGroup
import time


class RustBusterMain(Node):

	def __init__(self):
		super().__init__('rustbuster_main')
		# Subscribers
		self.switch = self.create_subscription(Bool, "rustbuster/explore", self.control, 1)
		self.detections_sub = self.create_subscription(AprilTagDetectionArray, "/detections", self.detections, 1)
		self.imu_sub = self.create_subscription(Imu, "/imu/data", self.cov_imu, 1)

		# Publishers
		self.explore = self.create_publisher(Bool, "explore/resume", 1)
		self.imu_pub = self.create_publisher(Imu, "my_imu", 10)

		# Odometry
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)
		self.odom_msg = Odometry()
		self.odom_msg_pub = self.create_publisher(Odometry, "odom", 10)
		self.odom_tf = TransformStamped()
		self.base_link_tf = TransformStamped()
		self.tf_pub = tf2_ros.TransformBroadcaster(self)


		e = Bool()
		e.data = True
		self.explore.publish(e) # start exploration here
		self.get_logger().info(20 * "#" + 2 * "\n#" + "\n\tstarting\t\n" + 2 * "\n#" + 20 * "#")

		#self.lease_sub = self.create_subscription(LeaseArray, "status/leases", self.request_spot_lease, 1)
		#self.claim_client = self.create_client(Trigger, "claim", callback_group=ReentrantCallbackGroup())

		self.timer = self.create_timer(0.1, self.main_loop)


	"""def request_spot_lease(self, lease_array):
		for resource in lease_array.resources:
			self.get_logger().info("lease client: %s" %resource.lease_owner.client_name)
			self.get_logger().info("lease user: %s" %resource.lease_owner.user_name)

			#clients
			#ros_spotgaming-3:spot_ros2-167892
			#bosdyn.android.spotapp bdb4de2162f9e6a8

			# get spot lease
			if not "ros_spotgaming-3:spot_ros2" in resource.lease_owner.client_name:
				t = 10
				self.get_logger().info("cut motor power or will regian lease in %ss" %str(t))
				time.sleep(t) # sleep ts to cut motor power before it tries to regain lease
				request = self.claim_client.call_async(Trigger.Request())
				#x = request.done()
				# change: /home/wizard/.local/lib/python3.10/site-packages/bosdyn/client/lease.py
				# so that it keeps the lease, forever
				#while not x:
				#	x = request.done()
					#self.get_logger().info("%s" %type(x))
				#print(request.result(), 300 * "-")
	"""

	def main_loop(self):
		#self.request_spot_lease()
		try:  # use spots visual odometry transform
			# tf2 odom
			odom_tf2 = self.tf_buffer.lookup_transform("vision", "camera_link", rclpy.time.Time())
			odom_tf2.header.stamp = self.get_clock().now().to_msg()
			odom_tf2.header.frame_id = "odom"
			odom_tf2.child_frame_id = "base_link"
			z = odom_tf2.transform.translation.z
			odom_tf2.transform.translation.z = 0.0
			self.tf_pub.sendTransform(odom_tf2)

			# base_link to be used by rtabmap must be a projection of body on the floor(Z=0)
			base_link_tf2 = TransformStamped()
			base_link_tf2.header.stamp = self.get_clock().now().to_msg()
			base_link_tf2.header.frame_id = "base_link"
			base_link_tf2.child_frame_id = "camera_link"
			base_link_tf2.transform.translation.z = z
			self.tf_pub.sendTransform(base_link_tf2)

		except TransformException as ex:
			self.get_logger().info(f'Could not transform {"vision"} to {"base_link"}: {ex}')

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
		new_imu.angular_velocity.x = imu.angular_velocity.z
		new_imu.angular_velocity.y = imu.angular_velocity.x
		new_imu.angular_velocity.z = imu.angular_velocity.y

		new_imu.linear_acceleration.x = imu.linear_acceleration.z
		new_imu.linear_acceleration.y = imu.linear_acceleration.x
		new_imu.linear_acceleration.z = imu.linear_acceleration.y

		self.imu_pub.publish(new_imu)

	def fake_odom(self, old_odom):
		# odom messages
		try:
			self.odom_msg = old_odom  # self.get_clock().now().to_msg()
			self.odom_msg.header.frame_id = "odom"
			self.odom_msg.child_frame_id = "base_link"  # TODO: make the correct twists
			odom_tf2 = self.tf_buffer.lookup_transform("vision", "base_link", rclpy.time.Time())
			self.odom_msg.pose.pose.position.x = odom_tf2.transform.translation.x
			self.odom_msg.pose.pose.position.y = odom_tf2.transform.translation.y
			self.odom_msg.pose.pose.position.z = odom_tf2.transform.translation.z
			self.odom_msg.pose.pose.orientation = odom_tf2.transform.rotation
			self.odom_msg_pub.publish(self.odom_msg)

		except TransformException as ex:
			self.get_logger().info(" trouble in fake odom..............")


	#def apriltag_detections
	def detections(self, dets):

		#for d in dets:
			#if not d in self.dets_list:
		if dets.detections:
			msg = "tag_%s\tt(ns)=%s\n" %(str(dets.detections[-1].id), str(self.get_clock().now().nanoseconds))
			# Append-adds at last
			file1 = open("tags_found.txt", "a")  # append mode
			file1.write(msg)
			file1.close()
			self.get_logger().info(msg)



def main(args=None):
	rclpy.init(args=args)
	rustbuster_main = RustBusterMain()
	rclpy.spin(rustbuster_main)
	rustbuster_main.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
