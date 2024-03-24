#!/usr/bin/python3
import numpy as np
import cv2
import depthai as dai
import matplotlib.pyplot as plt
from outlier_filter import *

pp = dai.Pipeline()
imu = pp.create(dai.node.IMU)
# create monocams because stereo wants them
left_lens = pp.createMonoCamera()
right_lens = pp.createMonoCamera()

# set resolution to 400p to save space
left_lens.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
right_lens.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

# create stereo depth
stereo = pp.createStereoDepth()
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setMedianFilter(dai.MedianFilter.MEDIAN_OFF)


rgbCamSocket = dai.CameraBoardSocket.CAM_B

stereo.initialConfig.setConfidenceThreshold(200)
stereo.setRectifyEdgeFillColor(0)
stereo.setLeftRightCheck(False)
stereo.setDepthAlign(rgbCamSocket)
xoutImu = pp.createXLinkOut()
xoutDepth = pp.createXLinkOut()
xoutLeft = pp.createXLinkOut()
xoutRight = pp.createXLinkOut()


xoutImu.setStreamName("imu")
xoutLeft.setStreamName("left")
xoutRight.setStreamName("right")
xoutDepth.setStreamName("depth")

# some stereo config stuff


imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_CALIBRATED, 10)
imu.enableIMUSensor(dai.IMUSensor.MAGNETOMETER_RAW, 10)
imu.enableIMUSensor(dai.IMUSensor.LINEAR_ACCELERATION, 10)

imu.setBatchReportThreshold(1)
imu.setMaxBatchReports(10)
# Link plugins IMU -> XLINK
imu.out.link(xoutImu.input)

#
left_lens.setBoardSocket(dai.CameraBoardSocket.CAM_B)
right_lens.setBoardSocket(dai.CameraBoardSocket.CAM_C)

#
left_lens.out.link(stereo.left)
right_lens.out.link(stereo.right)

#
stereo.disparity.link(xoutDepth.input)


cv2.startWindowThread()
cv2.namedWindow("scatter")
cv2.startWindowThread()
cv2.namedWindow("raw")

from messenger import *
from imu_messenger import *

ROS = True
RATE = 4
DEBUG = True


def main():
    val = np.zeros((400, 640)) + 10000
    narr = [val]
    if ROS:
        rospy.init_node("populate_laserscan", anonymous=True)
        # Create a publisher
        laserscan_pub = rospy.Publisher("/scan", LaserScan, queue_size=10)
        imu_pub = rospy.Publisher("/imu_oak", Imu, queue_size=10)

        # Set the loop rate
        rate = rospy.Rate(10)  # 10 Hz
    with dai.Device(pp) as device:
        qdepth = device.getOutputQueue(name="depth", maxSize=30, blocking=False)
        imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
        c = 0
        num = 882.5 * 7.5  # focal point * baseline for OAK-D

        while not rospy.is_shutdown() and True:
            # nonblocking try to get frames
            depthFrame = qdepth.tryGet()
            imuData = imuQueue.tryGet()
            if ROS and imuData != None:
                send_to_imu(imu_pub, imuData)

            if depthFrame != None:
                depthFrame = depthFrame.getFrame()

                depthFrame += 1
                depthFrame = num / depthFrame

                if c > 1:
                    narr[-1] = np.minimum(depthFrame, narr[-1])

                if c > 1 and not c % RATE:  # integrate over 25 frames
                    flipf = np.flip(narr[-1].T, axis=1)
                    rect_arr = find_obs(flipf)

                    # optional debugging
                    obs = get_free_space(rect_arr, flipf)

                    # "lidar"
                    markers = np.flip(get_markers(rect_arr))

                    if ROS:
                        x, dx = point_cloud(markers)
                        # Populate LaserScan message
                        laserscan_msg = populate_laserscan(x, dx)
                        # Publish the LaserScan message
                        laserscan_pub.publish(laserscan_msg)
                    # optional debugging
                    if DEBUG:
                        narr[-1] = np.minimum(narr[-1], np.flip(obs, axis=1).T)
                        cv2.imshow(
                            "raw", cv2.hconcat([depthFrame / 1000, narr[-1] / 1000])
                        )
                        cv2.imshow(
                            "scatter",
                            scatterplot(narr[-1].shape, markers[:, np.newaxis]),
                        )
                        cv2.waitKey(1)

                if c > 1 and not c % RATE:
                    narr = [val]

                c += 1


if __name__ == "__main__":
    main()
