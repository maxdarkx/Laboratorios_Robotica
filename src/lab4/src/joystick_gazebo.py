#!/usr/bin/env python
# encoding: utf-8
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import Joy
from lab4functions import *
from math import sqrt

def _map(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
    
# y = _map(25, 1, 50, 50, 1)
class Joystick(object):

    def __init__(self):
        self.axes = 6*[0.]
        self.buttons = 6*[0.]
        rospy.Subscriber("/joy", Joy, self.callback)
        print(rospy.Subscriber)

    def callback(self, msg):
        self.axes = msg.axes
        self.buttons = msg.buttons


if __name__ == '__main__':
    
    rospy.init_node("joystick_gazebo", disable_signals=True, anonymous=True)
    #rospy.init_node("joystick_node", anonymous=True)

    robot_client = actionlib.SimpleActionClient('follow_joint_trajectory',
                                                FollowJointTrajectoryAction)

    print "Waiting for server..."
    robot_client.wait_for_server()
    print "Connected to server"

    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    q1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    q4 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    x_cal=[0.0, 0.0, 0.0]
    joy_xdes = np.array([0.0, 0.1, 0.2])
    #q01 = np.array([0, 0, 0, 0, 0, 0])
    q01 = np.array([0.0, 1.57, 0.0, 1.57, 0.0, 0.0, 0.0, 0.0])# vector auxiliar para corregir offset 
    q02 = np.add(q0, q01)

    # Inverse kinematics
    q1[0:6] = ikine_ur5(joy_xdes, q02[0:6])
    # Resulting position (end effector with respect to the base link)
    #T = fkine_ur5(q1)


    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = joint_names

    # Initial position
    
    q2=np.add(q1,-q01)
    #g.trajectory.points = [ JointTrajectoryPoint(positions=q2, velocities=[0.0]*8, time_from_start=rospy.Duration(0.008))]
    #robot_client.send_goal(g)
    #robot_client.wait_for_result()
    
    joyS = Joystick()
    print("axes_I: ",joyS.axes)
    q3 = np.copy(q2)

    rospy.sleep(0.001)
    rate = rospy.Rate(30)


    while not rospy.is_shutdown():
        robot_client.cancel_goal()

        # Modification of the motion
        deadzone = 0.92
        x = round(joyS.axes[0]*deadzone,2)
        y = round(joyS.axes[1]*deadzone,2)
        z = round(joyS.axes[3]*deadzone,2)
        r = sqrt(x**2 + y**2 + z**2)
        
        if (0.3 <= r < 0.95):
            joy_xdes = [x,y,z]
            q4[0:6]= ikine_ur5(joy_xdes,q3[0:6])
            T = fkine_ur5(q4[0:6])
            #q4= np.add(q4,-q01)
            x_cal=T[0:3,3]
            g.trajectory.points = [ JointTrajectoryPoint(positions=(q4), velocities=[0]*8, time_from_start=rospy.Duration(0.008))]
            #q3= np.add(q4,-q01)
        print("Axes_F: ",np.round(q4[0:6],2), "Pos: ",np.round(joy_xdes,2)," Cal: ",np.round(x_cal,2))
     
        
        robot_client.send_goal(g)
        robot_client.wait_for_result()
        rate.sleep()
        
    robot_client.cancel_goal()