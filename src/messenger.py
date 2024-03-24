#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
import numpy as np


def point_cloud(array):
    mid = 290
    dx = 69 * np.pi / 180 / (2 * mid + 1)
    rect_arr = np.zeros((2 * mid, 1))

    for i in range(rect_arr.shape[0]):
        rect_arr[i, 0] = array[i]
    return rect_arr, dx


def populate_laserscan(msg, dx):
    # Create a LaserScan message
    laserscan_msg = LaserScan()

    # Populate header
    laserscan_msg.header.stamp = rospy.Time.now()
    laserscan_msg.header.frame_id = "front_laser"

    # Populate angle_min, angle_max, and angle_increment
    laserscan_msg.angle_min = -69 / 2 * np.pi / 180
    laserscan_msg.angle_max = 69 / 2 * np.pi / 180
    laserscan_msg.angle_increment = dx  # 1 degree resolution

    # Populate time_increment and scan_time
    laserscan_msg.time_increment = 0.0
    laserscan_msg.scan_time = 0.0
    vars = msg / 25
    # Populate range_min and range_max
    laserscan_msg.range_min = np.min(vars)
    laserscan_msg.range_max = np.max(vars)

    laserscan_msg.ranges = vars
    laserscan_msg.intensities = [0 for i in vars]
    return laserscan_msg


# def populate_imu(packet):
#     imu_msg = Imu()

#     # Header
#     imu_msg.header.stamp = rospy.Time.now()
#     imu_msg.header.frame_id = "base_link"  # Adjust the frame_id according to your setup

#     # Orientation
#     rotation_vector = packet.rotationVector
#     imu_msg.orientation = Quaternion(
#         rotation_vector.i, rotation_vector.j, rotation_vector.k, rotation_vector.real
#     )  # Set quaternion values (x, y, z, w)

#     # Angular velocity
#     gyroValues = packet.gyroscope
#     imu_msg.angular_velocity = Vector3(
#         gyroValues.x, gyroValues.y, gyroValues.z
#     )  # Set angular velocity values (x, y, z)

#     # Linear acceleration
#     linear_acceleration = packet.acceleroMeter
#     imu_msg.linear_acceleration = Vector3(
#         linear_acceleration.x, linear_acceleration.y, linear_acceleration.z
#     )
#     return imu_msg


if __name__ == "__main__":
    f = np.load("markers.npy")
    x, dx = point_cloud(f)
    x[x > 800] = 800
    rospy.init_node("populate_laserscan")

    # Create a publisher
    laserscan_pub = rospy.Publisher("/scan", LaserScan, queue_size=10)

    # Set the loop rate
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Populate LaserScan message
        laserscan_msg = populate_laserscan(x, dx)

        # Publish the LaserScan message
        laserscan_pub.publish(laserscan_msg)

        # Sleep to maintain the loop rate
        rate.sleep()
