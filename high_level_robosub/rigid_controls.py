#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from pynput import keyboard
import threading
import time
import math

def print_sim_time(self):
	sim_time = self.get_clock().now().to_msg()
	self.get_logger().info(f"Simulation Time: {sim_time.sec}.{sim_time.nanosec:09d}")

class TeleopPoseControl(Node):
	def __init__(self):
		super().__init__('pose_teleop')

		# Pose state
		self.x = 0.0
		self.y = 0.0
		self.z = 1.0
		self.roll = 0.0
		self.pitch = 0.0
		self.yaw = 0.0

		self.step_pos = 0.1
		self.step_rot = math.radians(5)

		self.model_name = 'high_level_robosub'
		topic_name = f"/model/{self.model_name}/pose"
		self.pose_pub = self.create_publisher(Pose, topic_name, 10)
	
		# Key press states
		self.key_state = set()

		# Start input thread
		self.running = True
		self.input_thread = threading.Thread(target=self.listen_keys)
		self.input_thread.start()

		# Timer for continuous updates
		self.timer = self.create_timer(0.05, self.update_pose)  # 20 Hz

	def listen_keys(self):
		def on_press(key):
			try:
				self.key_state.add(key.char)
			except AttributeError:
				pass

		def on_release(key):
			try:
				self.key_state.discard(key.char)
			except AttributeError:
				pass

		self.listener = keyboard.Listener(on_press=on_press, on_release=on_release)
		self.listener.start()

	def update_pose(self):
		sim_time = self.get_clock().now().to_msg()
		self.get_logger().info(f"Update Pose: {sim_time.sec}.{sim_time.nanosec:09d}")
		if 'w' in self.key_state: self.x += self.step_pos
		if 's' in self.key_state: self.x -= self.step_pos
		if 'a' in self.key_state: self.y += self.step_pos
		if 'd' in self.key_state: self.y -= self.step_pos
		if 'q' in self.key_state: self.z += self.step_pos
		if 'e' in self.key_state: self.z -= self.step_pos

		if 'i' in self.key_state: self.pitch += self.step_rot
		if 'k' in self.key_state: self.pitch -= self.step_rot
		if 'j' in self.key_state: self.yaw += self.step_rot
		if 'l' in self.key_state: self.yaw -= self.step_rot
		if 'u' in self.key_state: self.roll += self.step_rot
		if 'o' in self.key_state: self.roll -= self.step_rot

		pose = Pose()
		pose.position.x = self.x
		pose.position.y = self.y
		pose.position.z = self.z

		qx, qy, qz, qw = self.rpy_to_quaternion(self.roll, self.pitch, self.yaw)
		pose.orientation.x = qx
		pose.orientation.y = qy
		pose.orientation.z = qz
		pose.orientation.w = qw

		# self.get_logger().info(str(self.key_state))
		self.pose_pub.publish(pose)

	def rpy_to_quaternion(self, roll, pitch, yaw):
		# Standard conversion from RPY to quaternion
		cy = math.cos(yaw * 0.5)
		sy = math.sin(yaw * 0.5)
		cp = math.cos(pitch * 0.5)
		sp = math.sin(pitch * 0.5)
		cr = math.cos(roll * 0.5)
		sr = math.sin(roll * 0.5)

		qw = cr * cp * cy + sr * sp * sy
		qx = sr * cp * cy - cr * sp * sy
		qy = cr * sp * cy + sr * cp * sy
		qz = cr * cp * sy - sr * sp * cy
		return qx, qy, qz, qw

	def stop(self):
		self.running = False
		self.listener.stop()
		self.input_thread.join()

def main(args=None):
	rclpy.init(args=args)
	node = TeleopPoseControl()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		node.get_logger().info('Shutting down pose teleop.')
	node.stop()
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
