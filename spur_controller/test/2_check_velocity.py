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
# find maximum velocity of the robot

odom = {}
curr_pose = None
prev_pose = None
tm_prev = None

def odom_cb(msg):
    global odom
    odom[msg.header.frame_id] = msg
def pose_cb(msg):
    global curr_pose
    curr_pose = msg

def check_odom():
    global odom, curr_pose, prev_pose, tm_prev
    tm_curr = rospy.Time.now()
    t1_x = odom['map'].twist.twist.linear.x
    t1_y = odom['map'].twist.twist.linear.y
    t1_z = odom['map'].twist.twist.angular.z
    src = ""
    t2_x = t2_y = t2_z = 0
    if odom.has_key('odom'):
        t2_x = odom['odom'].twist.twist.linear.x
        t2_y = odom['odom'].twist.twist.linear.y
        t2_z = odom['odom'].twist.twist.angular.z
        src = "base "
    elif tm_prev and prev_pose:
        dt = (tm_curr - tm_prev).to_sec()
        t2_x = (curr_pose.x - prev_pose.x)/dt
        t2_y = (curr_pose.y - prev_pose.y)/dt
        t2_z = (curr_pose.theta  - prev_pose.theta)/dt
        src = "laser"
    tm_prev = tm_curr
    prev_pose = curr_pose
    print("ground_truth %7.3f %7.3f %7.3f, odom(%s) %7.3f %7.3f %7.3f, diff %7.3f %7.3f %7.3f" % (t1_x, t1_y, t1_z, src, t2_x, t2_y, t2_z, abs(t1_x-t2_x), abs(t1_y-t2_y), abs(t1_z-t2_z))) 
    return (t2_x, t2_y, t2_z)
    

if __name__=="__main__":
    # first thing, init a node!
    rospy.init_node('test_odom')

    # publish to cmd_vel
    p = rospy.Publisher('spur/cmd_vel', Twist)
    rospy.Subscriber('odom', Odometry, odom_cb, queue_size=10)
    rospy.Subscriber('base_pose_ground_truth', Odometry, odom_cb, queue_size=10)
    rospy.Subscriber('pose2D', Pose2D, pose_cb, queue_size=10)

    # create a twist message, fill in the details
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0

    linear_x = 0.1
    inc_x = 0.05
    angular_z = 0.2
    inc_z = 0.05
    sign = 1
    try:
        twist.linear.x = sign*linear_x
        twist.angular.z = 0
        while not rospy.is_shutdown():
            p.publish(twist)
            rospy.sleep(1)
            # announce move, and publish the message
            rospy.loginfo("move forward for 2 sec at %f"%twist.linear.x)
            for i in range(4):
                p.publish(twist)
                rospy.sleep(0.5) # 10*0.2 = 2.0
                diff = check_odom()
            print("diff %s"% str(diff))
            if abs(diff[0] - sign*linear_x) > 0.1:
                linear_x = linear_x - inc_x
                break
            linear_x = linear_x + inc_x
            sign = sign * -1
            twist.linear.x = sign*linear_x
        
        print("--- Maximum velocity is  %f"% linear_x)

        twist.linear.x = 0
        twist.angular.z = sign*angular_z
        while not rospy.is_shutdown() :
            p.publish(twist)
            rospy.sleep(1)
            # announce move, and publish the message
            rospy.loginfo("move forward for 2 sec at %f"%twist.linear.x)
            for i in range(4):
                p.publish(twist)
                rospy.sleep(0.5) # 10*0.2 = 2.0
                diff = check_odom()
            print("diff %s"% str(diff))
            if abs(diff[2] - sign*angular_z) > 0.1:
                angular_z = angular_z - inc_z
                break
            angular_z = angular_z + inc_z
            sign = sign * -1
            twist.angular.z = sign*angular_z

        
        print("--- Maximum angular is  %f"% angular_z)

    except rospy.ROSInterruptException:
        # create a new message
        twist = Twist()

        # note: everything defaults to 0 in twist, if we don't fill it in, we stop!
        rospy.loginfo("Stopping!")
        p.publish(twist)
    
        # twist.linear.x = 0
        # twist.angular.z = 0.3
        # rospy.loginfo("rotate for 5 sec (90 deg)")
        # for i in range(50):
        #     p.publish(twist)
        #     rospy.sleep(0.1) # 50*0.1 = 5.0
        #     print_odom()


