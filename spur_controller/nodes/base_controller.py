#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Tamagawa University. All rights reserved.
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
from math import sin, cos, atan2, hypot, fabs, sqrt, pi

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import quaternion_multiply, quaternion_about_axis, quaternion_matrix, translation_matrix
from sensor_msgs.msg import JointState


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
        self.sub_cmd_vel = rospy.Subscriber("cmd_vel", Twist, self.cmdCb)
        self.sub_state   = rospy.Subscriber("joint_states", JointState, self.stateCb)
        self.pub_bl_r = rospy.Publisher("bl_rotation_joint_position_controller/command", Float64, queue_size=1)
        self.pub_br_r = rospy.Publisher("br_rotation_joint_position_controller/command", Float64, queue_size=1)
        self.pub_fl_r = rospy.Publisher("fl_rotation_joint_position_controller/command", Float64, queue_size=1)
        self.pub_fr_r = rospy.Publisher("fr_rotation_joint_position_controller/command", Float64, queue_size=1)
        self.pub_bl_w = rospy.Publisher("bl_wheel_joint_velocity_controller/command", Float64, queue_size=1)
        self.pub_br_w = rospy.Publisher("br_wheel_joint_velocity_controller/command", Float64, queue_size=1)
        self.pub_fl_w = rospy.Publisher("fl_wheel_joint_velocity_controller/command", Float64, queue_size=1)
        self.pub_fr_w = rospy.Publisher("fr_wheel_joint_velocity_controller/command", Float64, queue_size=1)

        self.publish_odom = rospy.get_param('~publish_odom', True)
        if self.publish_odom:
            self.pub_odom = rospy.Publisher("odom", Odometry, queue_size=1)

        self.state = JointState()        # subscribed command
        self.cmd = Twist()        # subscribed command
        self.curr_cmd = self.cmd  # current target velocity
        self.last_cmd = self.cmd  # previous velocity
        self.cmd_time = rospy.Time()
        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.pose.pose.orientation.w = 1
        self.odom.pose.covariance[0] = self.odom.pose.covariance[7] = self.odom.pose.covariance[14] = self.odom.pose.covariance[21] = self.odom.pose.covariance[28] =  self.odom.pose.covariance[35] = 1
        self.odom.twist.covariance[0] = self.odom.twist.covariance[7] = self.odom.twist.covariance[14] = self.odom.twist.covariance[21] = self.odom.twist.covariance[28] =  self.odom.twist.covariance[35] = 1
        self.last_control_time = rospy.Time.now()

        self.x = 0
        self.y = 0
        self.th = 0

        rospy.on_shutdown(self.cleanup)

        rospy.sleep(1)
        rospy.loginfo("Start base controller (publish_odom: %s)"%(self.publish_odom))

    def cleanup(self):
        rospy.loginfo("Stop base controller")
        self.cmd_time = rospy.Time.now()
        self.last_cmd = Twist()
        self.control()
        time.sleep(1)

    def control(self, sec_idle=1.0):
        '''
        @brief Intended to be called periodically.
               Reads the last velocity command, interpretes for SPUR mechanism,
               then publish as Float64 that is the data type
               dynamixel_controllers receive.
        @type sec_idle: float
        @param sec_idle: Ideling seconds before robot stops moving
        '''
        control_interval = (rospy.Time.now() - self.last_control_time).to_sec()
        self.last_control_time = rospy.Time.now()

        if (rospy.Time.now() - self.cmd_time).to_sec() > sec_idle: ## if new cmd_vel did not comes for 5 sec
            self.cmd = Twist()

        ### velocity control (raw commnd velocity, we need to filter this)
        accel_trans_limit = 100
        accel_rotate_limit = 100
        raw_linear_x = self.cmd.linear.x  - self.last_cmd.linear.x
        raw_linear_y = self.cmd.linear.y  - self.last_cmd.linear.y
        raw_rotate  = self.cmd.angular.z - self.last_cmd.angular.z
        if control_interval > 0 and \
           (abs(raw_linear_x)/control_interval > accel_trans_limit or \
            abs(raw_linear_y)/control_interval > accel_trans_limit or \
            abs(raw_rotate)/control_interval > accel_rotate_limit) :
            rospy.logwarn("Too Large accel: cmd_vel %7.3f %7.3f %7.3f, cmd_vel_dot %7.3f %7.3f %7.3f" %
                          (self.cmd.linear.x, self.cmd.linear.y, self.cmd.angular.z,
                           abs(raw_linear_x)/control_interval,
                           abs(raw_linear_y)/control_interval,
                           abs(raw_rotate)/control_interval))

        self.curr_cmd.linear.x = self.last_cmd.linear.x + max(min(raw_linear_x, accel_trans_limit*control_interval), -accel_trans_limit*control_interval)
        self.curr_cmd.linear.y = self.last_cmd.linear.y + max(min(raw_linear_y, accel_trans_limit*control_interval), -accel_trans_limit*control_interval)
        self.curr_cmd.angular.z = self.last_cmd.angular.z + max(min(raw_rotate, accel_rotate_limit*control_interval), -accel_rotate_limit*control_interval)
        rospy.logdebug("cmd_vel %f %f %f" % (self.curr_cmd.linear.x, self.curr_cmd.linear.y, self.curr_cmd.angular.z))

        diameter = 0.1  # caster diameter
        offset_x = 0.15  # caster offset
        offset_y = 0.15
        #
        # http://www.chiefdelphi.com/media/papers/download/2614
        fr_v_x = self.curr_cmd.linear.x - self.curr_cmd.angular.z * (-offset_y)
        fr_v_y = self.curr_cmd.linear.y + self.curr_cmd.angular.z * (offset_x)
        fl_v_x = self.curr_cmd.linear.x - self.curr_cmd.angular.z * (offset_y)
        fl_v_y = self.curr_cmd.linear.y + self.curr_cmd.angular.z * (offset_x)
        br_v_x = self.curr_cmd.linear.x - self.curr_cmd.angular.z * (-offset_y)
        br_v_y = self.curr_cmd.linear.y + self.curr_cmd.angular.z * (-offset_x)
        bl_v_x = self.curr_cmd.linear.x - self.curr_cmd.angular.z * (offset_y)
        bl_v_y = self.curr_cmd.linear.y + self.curr_cmd.angular.z * (-offset_x)
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

        ## Odometry
        self.odom.header.stamp = rospy.Time.now()
        self.odom.header.frame_id = "odom"
        c_fr_v = c_fl_v = c_br_v = c_bl_v = c_fr_a = c_fl_a = c_br_a = c_bl_a = 0
        for i in range(len(self.state.name)):
            if self.state.name[i] == 'bl_rotation_joint':
                c_bl_a = self.state.position[i]*-1
            if self.state.name[i] == 'br_rotation_joint':
                c_br_a = self.state.position[i]*-1
            if self.state.name[i] == 'fl_rotation_joint':
                c_fl_a = self.state.position[i]*-1
            if self.state.name[i] == 'fr_rotation_joint':
                c_fr_a = self.state.position[i]*-1
            if self.state.name[i] == 'bl_wheel_joint':
                c_bl_v = self.state.velocity[i]*(diameter / 2.0)
                if abs(c_bl_v) < 0.002 :
                    c_bl_v = 0
            if self.state.name[i] == 'br_wheel_joint':
                c_br_v = self.state.velocity[i]*(diameter / 2.0)*-1
                if abs(c_br_v) < 0.002 :
                    c_br_v = 0
            if self.state.name[i] == 'fl_wheel_joint':
                c_fl_v = self.state.velocity[i]*(diameter / 2.0)
                if abs(c_fl_v) < 0.002 :
                    c_fl_v = 0
            if self.state.name[i] == 'fr_wheel_joint':
                c_fr_v = self.state.velocity[i]*(diameter / 2.0)*-1
                if abs(c_fr_v) < 0.002 :
                    c_fr_v = 0
        offset = sqrt(offset_x*offset_x + offset_y*offset_y)
        curr_linear_x =  (cos(c_fl_a)*c_fl_v + cos(c_fr_a)*c_fr_v + cos(c_br_a)*c_br_v + cos(c_bl_a)*c_bl_v)/4
        curr_linear_y =  (sin(c_fl_a)*c_fl_v + sin(c_fr_a)*c_fr_v + sin(c_br_a)*c_br_v + sin(c_bl_a)*c_bl_v)/4
        # rospy.loginfo("%f %f %f %f" % (c_fr_v*cos(pi/4-c_fr_a) , - c_fl_v*cos(-pi/4-c_fl_a),
        #                                c_br_v*cos(-pi/4-c_br_a), - c_bl_v*cos(pi/4-c_bl_a) ))
        # rospy.loginfo("%f %f %f %f" % (cos(pi/4-c_fr_a) , - cos(-pi/4-c_fl_a),
        #                                cos(-pi/4-c_br_a), - cos(pi/4-c_bl_a)))
        curr_angular_z = (c_fr_v*cos( pi/4-c_fr_a) - c_fl_v*cos(-pi/4-c_fl_a) +
                          c_br_v*cos(-pi/4-c_br_a) - c_bl_v*cos( pi/4-c_bl_a)) / (4*offset)

        # rospy.loginfo("wheel   %f %f %f %f" % (c_fr_v, c_fl_v, c_br_v, c_bl_v))
        # rospy.loginfo("rotate  %f %f %f %f" % (c_fr_a, c_fl_a, c_br_a, c_bl_a))
        # rospy.loginfo("curent cmd %f %f %f" % (curr_linear_x, curr_linear_y, curr_angular_z))
        delta_x = (curr_linear_x * cos(self.th) - curr_linear_y * sin(self.th)) * control_interval
        delta_y = (curr_linear_x * sin(self.th) + curr_linear_y * cos(self.th)) * control_interval
        delta_th = curr_angular_z * control_interval
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        # rospy.loginfo("                     -> %f %f %f\n", self.x, self.y, self.th)
        q = quaternion_about_axis(self.th, (0, 0, 1))
        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y
        self.odom.pose.pose.position.z = 0
        self.odom.pose.pose.orientation.x = q[0]
        self.odom.pose.pose.orientation.y = q[1]
        self.odom.pose.pose.orientation.z = q[2]
        self.odom.pose.pose.orientation.w = q[3]
        ##
        self.odom.twist.twist.linear.x = curr_linear_x
        self.odom.twist.twist.linear.y = curr_linear_y
        self.odom.twist.twist.angular.z = curr_angular_z

        if self.publish_odom:
            self.pub_odom.publish(self.odom)

        self.last_cmd = self.curr_cmd;

    def cmdCb(self, msg):
        self.cmd = msg
        self.cmd_time = rospy.Time.now()
        rospy.logdebug("cmd_vel %f %f %f" % (msg.linear.x, msg.linear.y, msg.angular.z))

    def stateCb(self, msg):
        self.state = msg

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = __doc__)
    # Accept control rate. Default is 100 hz.
    parser.add_argument("--rate", help="Controller rate", type=float, default=100.0)
    args, unknown = parser.parse_known_args()

    try:
        rospy.init_node("spur_base_controller")

        b = BaseController()
        rate = rospy.Rate(args.rate)
        sec_idle = rospy.get_param('/spur/spur_base_controller/sec_idle', 1.0)

        # Keep looping unless the system receives shutdown signal.
        while not rospy.is_shutdown():
            b.control(sec_idle)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
