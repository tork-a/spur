#!/usr/bin/env python

import rospy
from math import sin, pi
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from tf.transformations import euler_from_quaternion

#
# reset robot pose and restart spur_contrller to start from origin
#
# use laser: roslaunch base_controller.launch use_base_odom:=false
# use odom : roslaunch base_controller.launch use_base_odom:=true
# 
# with laser odom
# roslaunch spur_controller  spur_controller.launch use_base_odom:=false  use_dynamixel_controller:=false
# ground_truth  -0.143   0.556  -1.487, odom(laser)  -0.146   0.553  -1.493, diff   0.003   0.003   0.006
#
# without laser odom
# ground_truth  -0.065   0.179  -1.436, odom(base )   0.496  -0.007   0.097, diff   0.560   0.186   1.532


odom = {}
pose = None
def odom_cb(msg):
    global odom
    odom[msg.header.frame_id] = msg
def pose_cb(msg):
    global pose
    pose = msg

def print_odom():
    global odom, pose
    t1_x = odom['map'].pose.pose.position.x
    t1_y = odom['map'].pose.pose.position.y
    t1_z = euler_from_quaternion([odom['map'].pose.pose.orientation.x,odom['map'].pose.pose.orientation.y,odom['map'].pose.pose.orientation.z,odom['map'].pose.pose.orientation.w])[2]
    if odom.has_key('odom'):
        t2_x = odom['odom'].pose.pose.position.x
        t2_y = odom['odom'].pose.pose.position.y
        t2_z = euler_from_quaternion([odom['odom'].pose.pose.orientation.x,odom['odom'].pose.pose.orientation.y,odom['odom'].pose.pose.orientation.z,odom['odom'].pose.pose.orientation.w])[2]
        src = "base "
    else:
        t2_x = pose.x
        t2_y = pose.y
        t2_z = pose.theta
        src = "laser"
    print("ground_truth %7.3f %7.3f %7.3f, odom(%s) %7.3f %7.3f %7.3f, diff %7.3f %7.3f %7.3f" % (t1_x, t1_y, t1_z, src, t2_x, t2_y, t2_z, abs(t1_x-t2_x), abs(t1_y-t2_y), abs(t1_z-t2_z))) 

if __name__=="__main__":
    # first thing, init a node!
    rospy.init_node('test_odom')

    # publish to cmd_vel
    p = rospy.Publisher('spur/cmd_vel', Twist)
    rospy.Subscriber('odom', Odometry, odom_cb, queue_size=10)
    rospy.Subscriber('base_pose_ground_truth', Odometry, odom_cb, queue_size=10)
    rospy.Subscriber('pose2D', Pose2D, pose_cb, queue_size=10)

    # ..
    for i in range(50):
        rospy.sleep(0.1) # 50*0.1 = 5.0
        print_odom()

    # create a twist message, fill in the details
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0

    for i in range(4):
        # announce move, and publish the message
        twist.linear.x = 0.1
        twist.angular.z = 0
        rospy.loginfo("move forward for 5 sec (0.5m)")
        for i in range(50):
            p.publish(twist)
            rospy.sleep(0.1) # 50*0.1 = 5.0
            print_odom()

        twist.linear.x = 0
        twist.angular.z = 0.3
        rospy.loginfo("rotate for 5 sec (90 deg)")
        for i in range(50):
            p.publish(twist)
            rospy.sleep(0.1) # 50*0.1 = 5.0
            print_odom()

    # create a new message
    twist = Twist()

    # note: everything defaults to 0 in twist, if we don't fill it in, we stop!
    rospy.loginfo("Stopping!")
    p.publish(twist)
