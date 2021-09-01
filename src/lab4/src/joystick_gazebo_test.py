#!/usr/bin/env python
# encoding: utf-8
import time
from sensor_msgs.msg import Joy
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *

class Joystick(object):

    def __init__(self):
        self.axes = [0., 0., 0., 0., 0., 0.]
        self.buttons = [0., 0., 0., 0., 0., 0.]
        rospy.Subscriber("/joy", Joy, self.callback)

    def callback(self, msg):
        self.axes = msg.axes
        self.buttons = msg.buttons
        

if __name__ == '__main__':
    
    rospy.init_node("test2", disable_signals=True)

    robot_client = actionlib.SimpleActionClient('follow_joint_trajectory',
                                                FollowJointTrajectoryAction)

    print "Waiting for server..."
    robot_client.wait_for_server()
    print "Connected to server"

    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    Q0 = [0.0, -1.0, 1.7, -2.2, -1.6, 0.0, 0.0, 0.0]

    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = joint_names

    # Initial position
    g.trajectory.points = [ JointTrajectoryPoint(positions=Q0, velocities=[0]*8,
                                                 time_from_start=rospy.Duration(0.008))]
    robot_client.send_goal(g)
    robot_client.wait_for_result()
    rospy.sleep(0.1)
    joys=Joystick()

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        robot_client.cancel_goal()

        # Modification of the motion
        Q0 = joys.axes

        g.trajectory.points = [ JointTrajectoryPoint(positions=Q0, velocities=[0]*8,
                                                     time_from_start=rospy.Duration(0.008))]
        robot_client.send_goal(g)
        robot_client.wait_for_result()
        print("art: ",Q0)
        rate.sleep()

    robot_client.cancel_goal()

    
    
