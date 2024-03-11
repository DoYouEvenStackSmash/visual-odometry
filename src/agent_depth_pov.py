#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

raw_img = np.load("frames.npy")

## invert depth map
baseline = 7.5
focal_length = 882.5
max_disparity = 95
max_val = baseline*focal_length
camera_hfov = 69 * np.pi / 180

# get normalized disparity map to depth space
depth_img = ((baseline * focal_length)/((raw_img *  (baseline * focal_length) + 1) / (baseline*focal_length) * max_disparity + 1))

# normalize depth space
min_px = np.min(np.min(depth_img))
max_px = np.max(np.max(depth_img))
norm_depth_img = (depth_img - min_px) / (max_px - min_px)

#invert and shift
norm_depth_img = norm_depth_img * -1 + 1

# convert back to depth space
refined_img = norm_depth_img * (max_px - min_px)

## setup ##
# filters 
highp = np.array([0,0,-0.003,-0.0066, -0.163,1,0.163,0.066,0.003,0,0])	# high pass filter
lowp = np.array([0,0,1,1,1,1,1,1,1,1,1,0,0])														# low pass filter
deriv = np.array([0,1,-1,0])																						# differentiator

# n = 10
# nr = 1
# lowp = np.sinc(np.linspace(-nr, nr-1/n, n))														# sinc filter(unused)

arx = refined_img
a,b = arx.shape
w = 640
s = 0

# vectorized operations
hp = highp[:,np.newaxis]
lp = lowp[:,np.newaxis]
dp = deriv[:,np.newaxis]
conv = lambda x,h: np.apply_along_axis(lambda x: np.convolve(x, h.flatten(), mode='full'),axis=1,arr=x)
cdims = lambda a,x,h: a[:,h.shape[0]-1:h.shape[0]+1+x.shape[1]]

## signal path ### refined_img = arx -> arr -> xr -> fxx -> rx
# arr: high pass filtered input
# xr: first derivative of arr
# fxx: filtered first derivative of arr
# rx: second derivative of arr

arr = cdims(conv(np.transpose(arx),hp),np.transpose(arx), hp)/(arx.shape[0] + highp.shape[0])	# delay by highp.shape[0]
xr = cdims(conv(arr,dp),arr, dp)/(arr.shape[1]+deriv.shape[0])					# delay by deriv.shape[0]
fxx = cdims(conv(xr, lp),xr,lp)/(xr.shape[1] + lowp.shape[0])					# delay by lowp.shape[0]
rx = cdims(conv(fxx, dp),fxx,dp)/(fxx.shape[1]+deriv.shape[0])					# delay by deriv.shape[0]

# magic numbers for frame
s = 350
e = 50
d = 300

# fxx+rx -> rect_arr -> free_space

# detect obstacles
# the indexing madness is because i had to translate from matlab
def obs_detect(fxx, rx,avglen=4):
	fxx_delay = len(deriv)
	rx_delay = len(deriv) * 2 + 1
	xdot_sigma = np.std(fxx[50+fxx_delay:350+fxx_delay])
	xdotdot_sigma = np.std(rx[50+rx_delay:350+rx_delay])
	dot_crossing = np.zeros((d, 2))
	dot_crossing[:,0] = -1
	dot_arr = np.zeros(d) 
	flag = 0
	for i in range(1, d):
		# catch the knee of the floor
		if np.sign(np.mean(fxx[s-i:s-i+avglen])) == np.sign(dot_crossing[d - i, 0]) and flag < 1:
			dot_crossing[d - i-1 , 0] = np.mean(fxx[s-i:s-i+avglen])
		else:
			flag = 1
		# catch any vertical surfaces 
		if abs(np.mean(fxx[s-i:s-i+avglen])) - 6 * xdot_sigma > 0:
			dot_arr[d - i] = 1
	
	e = np.zeros(300)
	# set values such that vertical surfaces are infinitely tall
	counter = len(dot_arr)-1
	while counter != 0:
		if dot_arr[counter] > 0:
			break
		counter-=1
	dot_crossing[0:counter,0] = 0
	e = dot_crossing[:,0]
	e = (e - min(e)) / (max(e) - min(e))
	return e
	
# can this be vectorized?
rect_arr = np.array([obs_detect(fxx[i],rx[i],8) for i in range(w)])

# convert detected obstacles to free space
free_spaces = np.ones((640,300))
for i in range(w):
	flag = 0
	hi = 0
	hj = 0
 	# skip the lowest 10 rows because they are usually quite noisy
	for j in reversed(range(290,300)):
		free_spaces[i,j] = arr[i,j+49]
	for j in reversed(range(290)):
		if rect_arr[i,j] != 0 and flag != 1:
			free_spaces[i,j] = arr[i,j+49]
		else:
			if flag == 0:
				hi = i
				hj = j+1
			free_spaces[i,0:j] = free_spaces[hi,hj]
			break
			
plt.imshow(np.transpose(free_spaces))
plt.show() 