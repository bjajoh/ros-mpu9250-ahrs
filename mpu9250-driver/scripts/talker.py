#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu, MagneticField


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
    mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    imu_msg = Imu()
    mag_msg = MagneticField()
    rospy.loginfo("IMU STARTED")
    while not rospy.is_shutdown():
            # Fill mag msg
            mag_msg.header.stamp = rospy.get_rostime()
            mag_msg.magnetic_field.x = 0
            mag_msg.magnetic_field.y = 0
            mag_msg.magnetic_field.z = 0

            # create imu msg
            q0 = 1.0 #W
            q1 = 0.0 #X
            q2 = 0.0 #Y
            q3 = 0.0 #Z

            #Fill imu message
            imu_msg.header.stamp = rospy.get_rostime()
            imu_msg.header.frame_id = 'imu_raw'

            imu_msg.orientation.x = 0
            imu_msg.orientation.y = 0
            imu_msg.orientation.z = 0
            imu_msg.orientation.w = 0
            imu_msg.orientation_covariance[0] = 1e6
            imu_msg.orientation_covariance[0] = 1e6
            imu_msg.orientation_covariance[0] = 0.1

            imu_msg.angular_velocity.x = 0
            imu_msg.angular_velocity.y = 0
            imu_msg.angular_velocity.z = 0
            imu_msg.angular_velocity_covariance[0] = 1e6
            imu_msg.angular_velocity_covariance[4] = 1e6
            imu_msg.angular_velocity_covariance[8] = 0.1

            imu_msg.linear_acceleration.x = 0
            imu_msg.linear_acceleration.y = 0
            imu_msg.linear_acceleration.z = 0
            imu_msg.linear_acceleration_covariance[0] = 1e6
            imu_msg.linear_acceleration_covariance[4] = 1e6
            imu_msg.linear_acceleration_covariance[8] = 0.1

            imu_pub.publish(imu_msg)
            mag_pub.publish(mag_msg)
	    hello_str = "hello world %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
