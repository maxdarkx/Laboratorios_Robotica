#!/usr/bin/env python
import rospy
if __name__ == "__main__":
	rospy.init_node("node_hello")
print "Hello World UdeA!"
rospy.loginfo("This is a normal message :)")
rospy.logwarn("This is a warning :P")
rospy.logerr("This is an error message :(")
rospy.spin()
