#!/usr/bin/env python
# encoding: utf-8

import rospy
from sensor_msgs.msg import Joy


class Joystick(object):

    def __init__(self):
        self.axes = [0., 0., 0., 0., 0., 0.]
        self.buttons = [0., 0., 0., 0., 0., 0.]
        rospy.Subscriber("/joy", Joy, self.callback)

    def callback(self, msg):
        self.axes = msg.axes
        self.buttons = msg.buttons


rospy.init_node("testJoystick")

joystick = Joystick()

# Loop rate (in Hz)
rate = rospy.Rate(100)
# Continuous execution loop
while not rospy.is_shutdown():
    # Show the axes and buttons
    print '\naxis:', joystick.axes
    print 'buttons:', joystick.buttons
    # Wait for the next iteration
    rate.sleep()
