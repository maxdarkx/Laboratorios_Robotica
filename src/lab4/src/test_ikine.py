#!/usr/bin/env python
# encoding: utf-8

import rospy
from sensor_msgs.msg import JointState
from markers import *
from lab4functions import *


rospy.init_node("testInvKine")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)

bmarker      = BallMarker(color['RED'])
bmarker_des  = BallMarker(color['GREEN'])

# Joint names
jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# Desired position
xd = np.array([0, 0.4, 0.5])
# Initial configuration
q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q1 = np.array([0, +1.57, 0,+1.57, 0, 0])# vector auxiliar para corregir offset 
q2 = np.add(q0, q1)

# Inverse kinematics
q = ikine_ur5(xd, q2)

# Resulting position (end effector with respect to the base link)
T = fkine_ur5(q)
print('Obtained value:\n', np.round(T,3))

# Red marker shows the achieved position
bmarker.xyz(T[0:3,3])
print("Red marker: ", T[0:3,3])
# Green marker shows the desired position
bmarker_des.xyz(xd)
print("Green marker: ", xd)

# Objeto (mensaje) de tipo JointState
jstate = JointState()
# Asignar valores al mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = np.add(q, -q1)
print("joints:",jstate.position)
# Loop rate (in Hz)
rate = rospy.Rate(100)
# Continuous execution loop
while not rospy.is_shutdown():
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    # Publish the message
    pub.publish(jstate)
    bmarker.publish()
    bmarker_des.publish()
    # Wait for the next iteration
    rate.sleep()