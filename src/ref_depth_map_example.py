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
stereo.setConfidenceThreshold(150)
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
narr = [np.zeros((400,640))+1]
from skimage import feature
with dai.Device(pp) as device:
	qdepth = device.getOutputQueue(name="depth",maxSize=30,blocking=False)
	c = 0
	num = 882.5 * 7.5	# focal point * baseline for OAK-D
	norm = lambda depthFrame: (depthFrame - np.min(depthFrame)) / (np.max(depthFrame) - np.min(depthFrame))
	while True:
		# nonblocking try to get frames
		depthFrame = qdepth.tryGet()
		if depthFrame != None:
			depthFrame = depthFrame.getFrame()
			#narr[-1] = np.minimum(norm(depthFrame),narr[-1]) 
			#narr.append(depthFrame)
			depthFrame+=1
			print(np.min(depthFrame))
			depthFrame =  num / depthFrame
			if c < 100:
				narr[-1] = (np.minimum(norm(depthFrame),narr[-1]))
			else: 
				narr[-1] = (np.minimum(norm(depthFrame),narr[-1]))
			#depthFrame = (depthFrame - np.min(depthFrame)) / (np.max(depthFrame) - np.min(depthFrame))
			print(np.max(depthFrame.astype(np.float32)))
			#depthFrame[depthFrame<0.4] = 0.01
			#depthFrame = np.exp(-depthFrame*2)
			if False and c < 100 and not c % 20:			
				narr[-1] = cv2.bilateralFilter(narr[-1].astype(np.float32), d=9, sigmaColor=30,sigmaSpace=30)
			#plt.histogram(depthFrame)
			#plt.show()
			cv2.imshow("preview",narr[-1]*11)
			
			cv2.waitKey(1)
			c+=1
			if c > 200:
				#narr[-1] = cv2.bilateralFilter(narr[-1].astype(np.float32), d=9, sigmaColor=75,sigmaSpace=75)
				p = np.linspace(0,640,640)
				#plt.hist(narr[-1])
				#plt.hist(narr[-1][190:210,:]*40)
				#plt.show()
				np.save("frames.npy", narr[-1])
				exit()
				
		#print(c)
