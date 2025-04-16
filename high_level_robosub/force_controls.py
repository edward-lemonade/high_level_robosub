#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Wrench
from pynput import keyboard

pub = None
force_step = 5.0
torque_step = 1.0

def publish_wrench(force, torque):
    msg = Wrench()
    msg.force.x, msg.force.y, msg.force.z = force
    msg.torque.x, msg.torque.y, msg.torque.z = torque
    pub.publish(msg)

def on_press(key):
    try:
        f, t = [0, 0, 0], [0, 0, 0]

        if key.char == 'w': f[0] = force_step   # forward
        if key.char == 's': f[0] = -force_step  # backward
        if key.char == 'a': f[1] = force_step   # left
        if key.char == 'd': f[1] = -force_step  # right
        if key.char == 'q': f[2] = force_step   # up
        if key.char == 'e': f[2] = -force_step  # down

        if key.char == 'j': t[2] = torque_step  # yaw left
        if key.char == 'l': t[2] = -torque_step # yaw right
        if key.char == 'i': t[1] = torque_step  # pitch up
        if key.char == 'k': t[1] = -torque_step # pitch down
        if key.char == 'u': t[0] = torque_step  # roll left
        if key.char == 'o': t[0] = -torque_step # roll right

        publish_wrench(f, t)

    except AttributeError:
        pass

def main():
    global pub
    rospy.init_node('keyboard_force_controller')
    pub = rospy.Publisher('/block/cmd_force', Wrench, queue_size=1)

    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    rospy.spin()

if __name__ == '__main__':
    main()