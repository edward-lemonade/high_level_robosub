import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import math

class RigidControls(Node):
	def __init__(self):
		super().__init__('rigid_controls')
		self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
		self.create_subscription(Int32, '/keyboard/keypress', self.keypress_callback, 10)

		self.speed_linear = 0.5  # m/s
		self.speed_angular = 0.5  # rad/s

		self.get_logger().info("Keyboard teleop 6DoF node started.")

	def keypress_callback(self, msg: Int32):
		key_code = msg.data
		twist = Twist()

		# Map key codes to 6DoF motion

		if key_code == ord('W'):
			twist.linear.x = self.speed_linear
		elif key_code == ord('S'):
			twist.linear.x = -self.speed_linear
		elif key_code == ord('A'):
			twist.linear.y = self.speed_linear
		elif key_code == ord('D'):
			twist.linear.y = -self.speed_linear
		elif key_code == ord('R'):
			twist.linear.z = self.speed_linear
		elif key_code == ord('F'):
			twist.linear.z = -self.speed_linear
		elif key_code == ord('Q'):
			twist.angular.z = self.speed_angular
		elif key_code == ord('E'):
			twist.angular.z = -self.speed_angular
		elif key_code == ord('I'):
			twist.angular.x = self.speed_angular
		elif key_code == ord('K'):
			twist.angular.x = -self.speed_angular
		elif key_code == ord('J'):
			twist.angular.y = self.speed_angular
		elif key_code == ord('L'):
			twist.angular.y = -self.speed_angular
		else:
			# Unknown key
			return

		self.cmd_pub.publish(twist)
		#self.get_logger().info(f"Keypress {chr(key_code)} -> {twist}")

def main(args=None):
	rclpy.init(args=args)
	node = RigidControls()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
