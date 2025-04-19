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
        if key_code == ord('w'):
            twist.linear.x = self.speed_linear
        elif key_code == ord('s'):
            twist.linear.x = -self.speed_linear
        elif key_code == ord('a'):
            twist.linear.y = self.speed_linear
        elif key_code == ord('d'):
            twist.linear.y = -self.speed_linear
        elif key_code == ord('r'):
            twist.linear.z = self.speed_linear
        elif key_code == ord('f'):
            twist.linear.z = -self.speed_linear
        elif key_code == ord('q'):
            twist.angular.z = self.speed_angular
        elif key_code == ord('e'):
            twist.angular.z = -self.speed_angular
        elif key_code == ord('i'):
            twist.angular.x = self.speed_angular
        elif key_code == ord('k'):
            twist.angular.x = -self.speed_angular
        elif key_code == ord('j'):
            twist.angular.y = self.speed_angular
        elif key_code == ord('l'):
            twist.angular.y = -self.speed_angular
        else:
            # Unknown key
            return

        self.cmd_pub.publish(twist)
        self.get_logger().info(f"Keypress {chr(key_code)} -> {twist}")

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
