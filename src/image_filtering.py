#!/usr/bin/python3
import numpy as np
import cv2
import depthai as dai
import matplotlib.pyplot as plt
from outlier_filter import *
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
stereo.setMedianFilter(dai.MedianFilter.MEDIAN_OFF)

xoutDepth=pp.createXLinkOut()
xoutLeft=pp.createXLinkOut()
xoutRight = pp.createXLinkOut()

xoutLeft.setStreamName("left")
xoutRight.setStreamName("right")
xoutDepth.setStreamName("depth")

# some stereo config stuff
stereo.setConfidenceThreshold(150)
stereo.setRectifyEdgeFillColor(0)
stereo.setLeftRightCheck(False)

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

def scatterplot(image_shape, points, radius=3, color=(255, 255, 255)):
  # Create a blank image
  scatter_image = np.zeros(image_shape, dtype=np.uint8)
  # Draw circles for each point
  scale = np.max(points)
  scale_factor = image_shape[0] / scale
  
  for i,point in enumerate(points):
    #print(point)
    cv2.circle(scatter_image, (i,image_shape[0] - int(point[0]*scale_factor)), radius, color, -1)
  return scatter_image

# create preview windos
#cv2.startWindowThread()
#cv2.namedWindow("preview")
cv2.startWindowThread()
cv2.namedWindow("scatter")
cv2.startWindowThread()
cv2.namedWindow("raw")
#import skimage
val = np.zeros((400,640))+10000
narr = [val]
marr = [np.zeros((640,1)) + 10000]

frames = []
from skimage import feature
with dai.Device(pp) as device:
	qdepth = device.getOutputQueue(name="depth",maxSize=30,blocking=False)
	c = 0
	num = 882.5 * 7.5	# focal point * baseline for OAK-D
	norm = lambda depthFrame: depthFrame#(depthFrame - np.min(np.min(depthFrame))) / (np.max(np.max(depthFrame)) - np.min(np.min(depthFrame)))
	while True:
		# nonblocking try to get frames
		depthFrame = qdepth.tryGet()
		if depthFrame != None:
			depthFrame = depthFrame.getFrame()
			#cv2.imshow("raw",depthFrame)
			#cv2.waitKey(1)
			depthFrame+=1
			depthFrame =  num / depthFrame


			if c > 1:
				narr[-1] = np.minimum(depthFrame,narr[-1])


			if True and not c % 25:
				flipf = np.flip(narr[-1].T,axis=1)

				rect_arr = find_obs(flipf)
				obs = get_free_space(rect_arr, flipf)
				markers = get_markers(rect_arr)
				markers = flipf[np.arange(markers.shape[0]),markers.flatten()+50]
				
				#markers[markers>6000]=1
				#print(markers.shape)
				#marr[-1] = np.minimum(marr[-1],markers.reshape(-1,1))
				#print(marr[-1].shape)
				narr[-1] = np.minimum(narr[-1],np.flip(obs,axis=1).T)
				if c > 70:
					np.save("markers.npy",markers)
					np.save("scene.npy",narr[-1])
					exit()
				#cv2.imshow("scatter", scatterplot(narr[-1].shape, marr[-1]))
			#	cv2.waitKey(1)
				#cv2.imshow("preview",narr[-1]/1000)
				#cv2.waitKey(1)
				cv2.imshow("raw",cv2.hconcat([depthFrame/1000,narr[-1]/1000]))
				cv2.imshow("scatter",scatterplot(narr[-1].shape, markers[:,np.newaxis]))
				cv2.waitKey(1)
			if True and not c%25:
				narr = [val]
				#marr = [np.zeros((640,1)) + 10000]
			
			c+=1

				
		#print(c)
