#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Tokyo Opensource Robotics Kyokai Association.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the association nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
This script is used to profile the robot base. It commands a particular
velocity to the base_controller and then plots the actual outputs.
"""

import argparse
import time
import numpy
from math import atan2, hypot, fabs, pi

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
from std_msgs.msg import Float64
from tf.transformations import quaternion_multiply, quaternion_about_axis, quaternion_matrix, translation_matrix


class BaseController:
    '''
    Units:
      - x is m/s, accel_x is m/s^2
      - r is rad/s, accel_r is rad/s^2

    '''
    def __init__(self):
        '''
        @brief Instantiate each joint's publisher that publishes Float64 as
               position/velocity (velocity for wheels that rotate about pitch,
               position for yaw).
        '''
        self.sub = rospy.Subscriber("spur/cmd_vel", Twist, self.cmdCb)
        self.pub_bl_r = rospy.Publisher("bl_rotation_joint_position_controller/command", Float64, queue_size=1)
        self.pub_br_r = rospy.Publisher("br_rotation_joint_position_controller/command", Float64, queue_size=1)
        self.pub_fl_r = rospy.Publisher("fl_rotation_joint_position_controller/command", Float64, queue_size=1)
        self.pub_fr_r = rospy.Publisher("fr_rotation_joint_position_controller/command", Float64, queue_size=1)
        self.pub_bl_w = rospy.Publisher("bl_wheel_joint_velocity_controller/command", Float64, queue_size=1)
        self.pub_br_w = rospy.Publisher("br_wheel_joint_velocity_controller/command", Float64, queue_size=1)
        self.pub_fl_w = rospy.Publisher("fl_wheel_joint_velocity_controller/command", Float64, queue_size=1)
        self.pub_fr_w = rospy.Publisher("fr_wheel_joint_velocity_controller/command", Float64, queue_size=1)
        self.pub_odom = rospy.Publisher("odom", Odometry, queue_size=1)

        self.last_cmd_msg = Twist()
        self.last_cmd_time = rospy.Time()
        self.cmd = self.last_cmd_msg
        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.pose.pose.orientation.w = 1
        self.odom.pose.covariance[0] = self.odom.pose.covariance[7] = self.odom.pose.covariance[14] = self.odom.pose.covariance[21] = self.odom.pose.covariance[28] =  self.odom.pose.covariance[35] = 1
        self.odom.twist.covariance[0] = self.odom.twist.covariance[7] = self.odom.twist.covariance[14] = self.odom.twist.covariance[21] = self.odom.twist.covariance[28] =  self.odom.twist.covariance[35] = 1
        self.last_control_time = rospy.Time.now()

        rospy.on_shutdown(self.cleanup)

        rospy.sleep(1)
        rospy.loginfo("Start base controller")

    def cleanup(self):
        rospy.loginfo("Stop base controller")
        self.last_cmd_time = rospy.Time.now()
        self.last_cmd_msg = Twist()
        self.control()
        time.sleep(1)

    def control(self):
        '''
        @brief Intended to be called periodically.
               Reads the last velocity command, interpretes for SPUR mechanism,
               then publish as Float64 that is the data type
               dynamixel_controllers receive.
        '''
        control_interval = (rospy.Time.now() - self.last_control_time).to_sec()
        self.last_control_time = rospy.Time.now()

        v1 = translation_matrix((self.cmd.linear.x * control_interval, self.cmd.linear.y * control_interval, 0))
        o  = self.odom.pose.pose.orientation
        v2 = numpy.dot(quaternion_matrix([o.x, o.y, o.z, o.w]), v1)
        self.odom.pose.pose.position.x += v2[0,3]
        self.odom.pose.pose.position.y += v2[1,3]
        q1 = quaternion_about_axis(self.cmd.angular.z*control_interval, (0, 0, 1))
        o  = self.odom.pose.pose.orientation
        q2 = quaternion_multiply([o.x, o.y, o.z, o.w], q1)
        self.odom.pose.pose.orientation.x = q2[0]
        self.odom.pose.pose.orientation.y = q2[1]
        self.odom.pose.pose.orientation.z = q2[2]
        self.odom.pose.pose.orientation.w = q2[3]
        self.odom.twist.twist.linear.x = v2[0,3]
        self.odom.twist.twist.linear.y = v2[1,3]
        self.odom.twist.twist.angular.z += self.cmd.angular.z * control_interval
        self.odom.header.stamp = rospy.Time.now()

        if (rospy.Time.now() - self.last_cmd_time).to_sec() > 5: ## if new cmd_vel did not comes for 5 sec
            self.last_cmd_msg = Twist()

        velocity_limit = 0.001
        self.cmd.linear.x += max(min(self.last_cmd_msg.linear.x - self.cmd.linear.x, velocity_limit), -velocity_limit)
        self.cmd.linear.y += max(min(self.last_cmd_msg.linear.y - self.cmd.linear.y, velocity_limit), -velocity_limit)
        self.cmd.angular.z += max(min(self.last_cmd_msg.angular.z - self.cmd.angular.z, velocity_limit), -velocity_limit)
        rospy.logdebug("cmd_vel %f %f %f" % (self.cmd.linear.x, self.cmd.linear.y, self.cmd.angular.z))

        diameter = 0.1  # caster diameter
        offset_x = 0.15  # caster offset
        offset_y = 0.15
        #
        # http://www.chiefdelphi.com/media/papers/download/2614
        fr_v_x = self.cmd.linear.x - self.cmd.angular.z * (-offset_y)
        fr_v_y = self.cmd.linear.y + self.cmd.angular.z * (offset_x)
        fl_v_x = self.cmd.linear.x - self.cmd.angular.z * (offset_y)
        fl_v_y = self.cmd.linear.y + self.cmd.angular.z * (offset_x)
        br_v_x = self.cmd.linear.x - self.cmd.angular.z * (-offset_y)
        br_v_y = self.cmd.linear.y + self.cmd.angular.z * (-offset_x)
        bl_v_x = self.cmd.linear.x - self.cmd.angular.z * (offset_y)
        bl_v_y = self.cmd.linear.y + self.cmd.angular.z * (-offset_x)
        # v[m/s] = r[rad/s] * 0.1[m]  ## 0.1 = diameter
        fr_v = hypot(fr_v_x, fr_v_y)
        fl_v = hypot(fl_v_x, fl_v_y)
        br_v = hypot(br_v_x, br_v_y)
        bl_v = hypot(bl_v_x, bl_v_y)
        fr_a = atan2(fr_v_y, fr_v_x)
        fl_a = atan2(fl_v_y, fl_v_x)
        br_a = atan2(br_v_y, br_v_x)
        bl_a = atan2(bl_v_y, bl_v_x)

        # fix for -pi/2 - pi/2
        fr_v = -fr_v if fabs(fr_a) > pi / 2 else fr_v
        fr_a = fr_a - pi if fr_a > pi / 2 else fr_a
        fr_a = fr_a + pi if fr_a < -pi / 2 else fr_a
        fl_v = -fl_v if fabs(fl_a) > pi / 2 else fl_v
        fl_a = fl_a - pi if fl_a > pi / 2 else fl_a
        fl_a = fl_a + pi if fl_a < -pi / 2 else fl_a
        br_v = -br_v if fabs(br_a) > pi / 2 else br_v
        br_a = br_a - pi if br_a > pi / 2 else br_a
        br_a = br_a + pi if br_a < -pi / 2 else br_a
        bl_v = -bl_v if fabs(bl_a) > pi / 2 else bl_v
        bl_a = bl_a - pi if bl_a > pi / 2 else bl_a
        bl_a = bl_a + pi if bl_a < -pi / 2 else bl_a

        rospy.logdebug("wheel   %f %f %f %f" % (fr_v, fl_v, br_v, bl_v))
        rospy.logdebug("rotate  %f %f %f %f" % (fr_a, fl_a, br_a, bl_a))

        self.pub_fr_w.publish(Float64(-1*fr_v/(diameter / 2.0)))  # right wheel has -1 axis orientation
        self.pub_fl_w.publish(Float64(   fl_v/(diameter / 2.0)))
        self.pub_br_w.publish(Float64(-1*br_v/(diameter / 2.0)))
        self.pub_bl_w.publish(Float64(   bl_v/(diameter / 2.0)))
        self.pub_fr_r.publish(Float64(-1*fr_a))  # All steerage wheels' rotational direction needs
        self.pub_fl_r.publish(Float64(-1*fl_a))  # to be negated since they are upside down.
        self.pub_br_r.publish(Float64(-1*br_a))
        self.pub_bl_r.publish(Float64(-1*bl_a))
        self.pub_odom.publish(self.odom)

    def cmdCb(self, msg):
        self.last_cmd_msg = msg
        self.last_cmd_time = rospy.Time.now()
        rospy.logdebug("cmd_vel %f %f %f" % (msg.linear.x, msg.linear.y, msg.angular.z))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = __doc__)
    # Accept control rate. Default is 100 hz.
    parser.add_argument("--rate", help="Controller rate", type=float, default=100.0)
    args, unknown = parser.parse_known_args()

    try:
        rospy.init_node("spur_base_controller")

        b = BaseController()
        rate = rospy.Rate(args.rate)

        # Keep looping unless the system receives shutdown signal.
        while not rospy.is_shutdown():
            b.control()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
