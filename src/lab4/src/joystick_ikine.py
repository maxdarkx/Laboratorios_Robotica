#!/usr/bin/env python
# encoding: utf-8

import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from markers import *
from lab4functions import *
from math import sqrt

class Joystick(object):

    def __init__(self):
        self.axes = [0., 0., 0., 0., 0., 0.]
        self.buttons = [0., 0., 0., 0., 0., 0.]
        rospy.Subscriber("/joy", Joy, self.callback)

    def callback(self, msg):
        self.axes = msg.axes
        self.buttons = msg.buttons

once = True
rospy.init_node("joyIkine")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)

bmarker      = BallMarker(color['RED'])
bmarker_des  = BallMarker(color['GREEN'])

# Joint names
jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# Desired position
joyS = Joystick()
xd = np.array([0.0, 0.0, 0.4])

# Initial configuration
q0 = np.array([0.83, -1.05, 3.4, 2.15, -3.07, 0.0])

if once:
    q01 = np.array([0, +1.57, 0,+1.57, 0, 0])# vector auxiliar para corregir offset 
    q02 = np.add(q0, q01)
    once = False


# Inverse kinematics
q1 = ikine_ur5(xd, q02)

# Resulting position (end effector with respect to the base link)
T = fkine_ur5(q1)
print('Obtained value:\n', np.round(T,3))

# Red marker shows the achieved position
point = T[0:3,3]
bmarker.xyz(point)
print("Red marker: \n", point)
# Green marker shows the desired position
bmarker_des.xyz(xd)
#print("Green marker: \n", xd)

# Objeto (mensaje) de tipo JointState
jstate = JointState()
# Asignar valores al mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = np.add(q1,-q01)


# Loop rate (in Hz)
rate = rospy.Rate(120)
# Continuous execution loop

while not rospy.is_shutdown():
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    
    # message
    deadzone = 0.92
    x = round(joyS.axes[0]*deadzone,2)
    y = round(joyS.axes[1]*deadzone,2)
    z = round((-(joyS.axes[2]-1)/2)*deadzone,2)
    r = sqrt(x**2 + y**2 + z**2)
    
    if (0.2 <= r < 1):
        xd = [x, y, z]
        q2= ikine_ur5(xd,q1)
        T = fkine_ur5(q2)
        q2=np.add(q2,-q01)
        point = T[0:3,3]
        bmarker.xyz(point)
        bmarker_des.xyz(xd)

        # Publish the message
        jstate.position = q2 
        pub.publish(jstate)
        bmarker.publish()
        bmarker_des.publish()
    # Wait for the next iteration
    print(np.round(jstate.position,2),np.round(xd,2))
    print(np.round(r,2))
    rate.sleep()
