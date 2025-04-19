import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from ros_gz_interfaces.srv import SetEntityPose
from geometry_msgs.msg import Pose, Point, Quaternion
import math
import tf_transformations
import time

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

        # Integrate angular motion (Euler angles -> quaternion)
        roll, pitch, yaw = tf_transformations.euler_from_quaternion([
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w or 1.0
        ])

        roll += self.current_twist.angular.x * dt
        pitch += self.current_twist.angular.y * dt
        yaw += self.current_twist.angular.z * dt

        quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        self.current_pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )

        # Send updated pose to Gazebo
        req = SetEntityPose.Request()
        req.pose = self.current_pose
        req.name = "high_level_robosub"
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
