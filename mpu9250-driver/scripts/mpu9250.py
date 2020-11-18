#!/usr/bin/env python
# Bjarne Johannsen, 2020


import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu, MagneticField
import math

import time
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

G = 9.81
MagFieldConversion = 1000

mpu = MPU9250(
    address_ak=AK8963_ADDRESS,
    address_mpu_master=MPU9050_ADDRESS_68, # In 0x68 Address
    address_mpu_slave=None,
    bus=1,
    gfs=GFS_1000,
    afs=AFS_8G,
    mfs=AK8963_BIT_16,
    mode=AK8963_MODE_C100HZ)

def talker():
    imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
    mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    imu_msg = Imu()
    mag_msg = MagneticField()

    # Apply the settings to the registers and calibrate
    mpu.configure()
    mpu.calibrate()
    mpu.configure() 

    rospy.loginfo("IMU STARTED")
    while not rospy.is_shutdown():
            # Fill mag msg
            mx, my, mz = mpu.readMagnetometerMaster()
            mag_msg.header.stamp = rospy.get_rostime()
            mag_msg.magnetic_field.x = mx/MagFieldConversion
            mag_msg.magnetic_field.y = my/MagFieldConversion
            mag_msg.magnetic_field.z = mz/MagFieldConversion

            # create imu msg
            q0 = 1.0 #W
            q1 = 0.0 #X
            q2 = 0.0 #Y
            q3 = 0.0 #Z

            #Fill imu message
            imu_msg.header.stamp = rospy.get_rostime()
            imu_msg.header.frame_id = 'imu_frame'

            imu_msg.orientation.x = q0
            imu_msg.orientation.y = q1
            imu_msg.orientation.z = q2
            imu_msg.orientation.w = q3
            imu_msg.orientation_covariance[0] = 0.1
            imu_msg.orientation_covariance[0] = 0.1
            imu_msg.orientation_covariance[0] = 0.1

            gx, gy, gz = mpu.readGyroscopeMaster()
            imu_msg.angular_velocity.x = math.radians(gx)
            imu_msg.angular_velocity.y = math.radians(gy)
            imu_msg.angular_velocity.z = math.radians(gz)
            imu_msg.angular_velocity_covariance[0] = 0.1
            imu_msg.angular_velocity_covariance[4] = 0.1
            imu_msg.angular_velocity_covariance[8] = 0.1

            ax, ay, az = mpu.readAccelerometerMaster()
            imu_msg.linear_acceleration.x = ax*G
            imu_msg.linear_acceleration.y = ay*G
            imu_msg.linear_acceleration.z = az*G
            imu_msg.linear_acceleration_covariance[0] = 0.1
            imu_msg.linear_acceleration_covariance[4] = 0.1
            imu_msg.linear_acceleration_covariance[8] = 0.1

            imu_pub.publish(imu_msg)
            mag_pub.publish(mag_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
