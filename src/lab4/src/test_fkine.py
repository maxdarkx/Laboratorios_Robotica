#!/usr/bin/env python
# encoding: utf-8

import rospy
from sensor_msgs.msg import JointState

from markers import *
from lab4functions import *


rospy.init_node("testForwardKinematics")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
bmarker = BallMarker(color['GREEN'])

# Joint names
jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# Joint Configuration

q = np.array([0, -0.785, 0, 0, 0, 0])
q1 = np.array([0, +1.57, 0,+1.57, 0, 0])# vector auxiliar para corregir offset 
q2 = np.add(q, q1)


# End effector with respect to the base
T = fkine_ur5(q2)
print( np.round(T, 3) )
bmarker.position(T)

# Object (message) whose type is JointState
jstate = JointState()
# Set values to the message
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = q

# Loop rate (in Hz)
rate = rospy.Rate(100)
# Continuous execution loop
while not rospy.is_shutdown():
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    # Publish the message
    pub.publish(jstate)
    bmarker.publish()
    # Wait for the next iteration
    rate.sleep()