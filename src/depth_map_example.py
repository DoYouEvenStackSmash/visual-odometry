#!/usr/bin/python3
import numpy as np
import cv2
import depthai as dai
import matplotlib.pyplot as plt

pp = dai.Pipeline()

# create monocams because stereo wants them
left_lens = pp.createMonoCamera()
right_lens = pp.createMonoCamera()

# set resolution to 400p to save space
left_lens.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
right_lens.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

# create stereo depth
stereo = pp.createStereoDepth()
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setMedianFilter(dai.MedianFilter.KERNEL_7x7)

xoutDepth=pp.createXLinkOut()
xoutLeft=pp.createXLinkOut()
xoutRight = pp.createXLinkOut()

xoutLeft.setStreamName("left")
xoutRight.setStreamName("right")
xoutDepth.setStreamName("depth")

# some stereo config stuff
stereo.setConfidenceThreshold(200)
stereo.setRectifyEdgeFillColor(0)
stereo.setLeftRightCheck(True)

#
left_lens.setBoardSocket(dai.CameraBoardSocket.LEFT)
right_lens.setBoardSocket(dai.CameraBoardSocket.RIGHT)

#
left_lens.out.link(stereo.left)
right_lens.out.link(stereo.right)

#
stereo.disparity.link(xoutDepth.input)

#not useful, runs us out of queue space
#ll.out.link(xoutLeft.input)
#rr.out.link(xoutRight.input)

# create preview windos
cv2.startWindowThread()
cv2.namedWindow("preview")
#import skimage
narr = []
from skimage import feature
with dai.Device(pp) as device:
	qdepth = device.getOutputQueue(name="depth",maxSize=30,blocking=False)
	c = 0
	num = 882.5 * 7.5	# focal point * baseline for OAK-D
	while True:
		# nonblocking try to get frames
		depthFrame = qdepth.tryGet()
		if depthFrame != None:
			depthFrame = depthFrame.getFrame()
			narr.append(depthFrame)
			depthFrame+=1
			print(np.min(depthFrame))
			depthFrame =  num / depthFrame
			depthFrame = (depthFrame - np.min(depthFrame)) / (np.max(depthFrame) - np.min(depthFrame))
			print(np.max(depthFrame.astype(np.float32)))
			#depthFrame[depthFrame<0.4] = 0.01
			#depthFrame = np.exp(-depthFrame*2)
			#depthFrame = cv2.bilateralFilter(depthFrame.astype(np.float32), d=9, sigmaColor=75,sigmaSpace=75)
			#cv2.imshow("preview",depthFrame.astype(np.float32))
			#c+=1
			#if c > 125:
			#	np.save("frames.npy", np.array(narr))
			#	exit()

			cv2.waitKey(1)
		#print(c)
