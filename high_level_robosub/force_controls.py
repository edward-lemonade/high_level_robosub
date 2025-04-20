import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from gazebo_msgs.srv import ApplyLinkWrench
import math

class ForceControls(Node):
	force_magnitude = 10

	def __init__(self):
		super().__init__('force_controls')

		self.client = self.create_client(ApplyLinkWrench, '/gazebo/apply_link_wrench')
		while not self.client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Waiting for /gazebo/apply_link_wrench service...')

		self.force = [0.0, 0.0, 0.0]  
		self.create_subscription(Int32, '/keyboard/keypress', self.keypress_callback, 10)

		self.link_name = 'high_level_robosub::base_link'  # Change to your model and link

	def keypress_callback(self, msg: Int32):
		key_code = msg.data
		
		req = ApplyLinkWrench.Request()
		req.link_name = self.link_name

		req.wrench.force.x = self.force[0]
		req.wrench.force.y = self.force[1]
		req.wrench.force.z = self.force[2]

		if key_code == ord('W'):
			self.force[0] = self.force_magnitude
		elif key_code == ord('S'):
			self.force[0] = -self.force_magnitude
		elif key_code == ord('A'):
			self.force[1] = self.force_magnitude
		elif key_code == ord('D'):
			self.force[1] = -self.force_magnitude
		elif key_code == ord('V'):
			self.force[2] = self.force_magnitude
		elif key_code == ord(' '):
			self.force[2] = -self.force_magnitude
		else:
			# Unknown key
			return

		future = self.client.call_async(req)
		#self.get_logger().info(f"Keypress {chr(key_code)} -> {twist}")

def main(args=None):
	rclpy.init(args=args)
	node = ForceControls()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
