#!/usr/bin/env python

import rospy
from math import sin, pi
from geometry_msgs.msg import Twist

speed_trans = 0.1
speed_rotate = 0.5

if __name__=="__main__":

    # first thing, init a node!
    rospy.init_node('cmd_vel_test')

    # publish to cmd_vel
    p = rospy.Publisher('spur/cmd_vel', Twist)

    # create a twist message, fill in the details
    twist = Twist()
    twist.linear.x = speed_trans
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0

    # announce move, and publish the message
    rospy.loginfo("About to be moving forward!")
    for i in range(30):
        p.publish(twist)
        rospy.sleep(0.1) # 30*0.1 = 3.0

    twist.linear.x = speed_trans
    twist.linear.y = 0
    twist.angular.z = speed_trans
    rospy.loginfo("About to be moving forward + rotation")
    for i in range(30):
        p.publish(twist)
        rospy.sleep(0.1) # 30*0.1 = 3.0

    twist.linear.x = speed_trans
    twist.linear.y = 0
    twist.angular.z = -speed_trans
    rospy.loginfo("About to be moving forward - rotation")
    for i in range(30):
        p.publish(twist)
        rospy.sleep(0.1) # 30*0.1 = 3.0

    twist.linear.x = 0
    twist.linear.y = speed_trans
    rospy.loginfo("About to be moving to right!")
    for i in range(30):
        p.publish(twist)
        rospy.sleep(0.1) # 30*0.1 = 3.0

    twist.linear.x = 0
    twist.linear.y = 0
    twist.angular.z = speed_trans
    rospy.loginfo("About to be moving rotation!")
    for i in range(30):
        p.publish(twist)
        rospy.sleep(0.1) # 30*0.1 = 3.0

    twist.linear.x = speed_trans
    twist.linear.y = 0
    rospy.loginfo("About to be moving sin curve")
    for i in range(100):
        twist.angular.z = speed_rotate*sin(pi*i/50.0)
        rospy.loginfo("%6.4f %6.4f %6.4f" % (twist.linear.x, twist.linear.y, twist.angular.z))
        p.publish(twist)
        rospy.sleep(0.1) # 30*0.1 = 3.0

    # create a new message
    twist = Twist()

    # note: everything defaults to 0 in twist, if we don't fill it in, we stop!
    rospy.loginfo("Stopping!")
    p.publish(twist)
