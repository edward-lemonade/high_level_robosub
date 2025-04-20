import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from ros_gz_interfaces.srv import SetEntityPose
from geometry_msgs.msg import Pose, Point, Quaternion
import math
import time

def euler_from_quaternion(q: Quaternion):
	"""Convert quaternion to roll, pitch, yaw (in radians)."""
	x, y, z, w = q.x, q.y, q.z, q.w or 1.0

	# Roll (x-axis rotation)
	sinr_cosp = 2 * (w * x + y * z)
	cosr_cosp = 1 - 2 * (x * x + y * y)
	roll = math.atan2(sinr_cosp, cosr_cosp)

	# Pitch (y-axis rotation)
	sinp = 2 * (w * y - z * x)
	if abs(sinp) >= 1:
		pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
	else:
		pitch = math.asin(sinp)

	# Yaw (z-axis rotation)
	siny_cosp = 2 * (w * z + x * y)
	cosy_cosp = 1 - 2 * (y * y + z * z)
	yaw = math.atan2(siny_cosp, cosy_cosp)

	return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
	"""Convert roll, pitch, yaw (in radians) to a quaternion."""
	cy = math.cos(yaw * 0.5)
	sy = math.sin(yaw * 0.5)
	cp = math.cos(pitch * 0.5)
	sp = math.sin(pitch * 0.5)
	cr = math.cos(roll * 0.5)
	sr = math.sin(roll * 0.5)

	q = Quaternion()
	q.w = cr * cp * cy + sr * sp * sy
	q.x = sr * cp * cy - cr * sp * sy
	q.y = cr * sp * cy + sr * cp * sy
	q.z = cr * cp * sy - sr * sp * cy
	return q

class TwistToPoseNode(Node):
	def __init__(self):
		super().__init__('twist_to_pose_updater')

		# Internal state
		self.current_pose = Pose()
		self.last_update_time = self.get_clock().now()
		self.current_twist = Twist()

		# Subscriber to Twist
		self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)

		# Timer for integration
		self.timer = self.create_timer(0.05, self.update_pose)

		# Service client
		self.cli = self.create_client(SetEntityPose, '/gazebo/set_entity_pose')
		while not self.cli.wait_for_service(timeout_sec=3.0):
			self.get_logger().info('Waiting for /gazebo/set_entity_pose service...')

		self.get_logger().info("TwistToPoseNode started")

	def twist_callback(self, msg: Twist):
		self.current_twist = msg

	def update_pose(self):
		now = self.get_clock().now()
		dt = (now - self.last_update_time).nanoseconds * 1e-9
		self.last_update_time = now

		# Integrate linear motion
		self.current_pose.position.x += self.current_twist.linear.x * dt
		self.current_pose.position.y += self.current_twist.linear.y * dt
		self.current_pose.position.z += self.current_twist.linear.z * dt

		# Convert quaternion to Euler angles
		roll, pitch, yaw = euler_from_quaternion(self.current_pose.orientation)

		# Integrate angular velocity
		roll += self.current_twist.angular.x * dt
		pitch += self.current_twist.angular.y * dt
		yaw += self.current_twist.angular.z * dt

		self.current_pose.orientation = quaternion_from_euler(roll, pitch, yaw)

		# Send updated pose to Gazebo
		req = SetEntityPose.Request()
		req.pose = self.current_pose

		self.get_logger().info(str(req.pose))

		self.cli.call_async(req)


def main(args=None):
	rclpy.init(args=args)
	node = TwistToPoseNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
