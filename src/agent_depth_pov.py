#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

raw_img = np.load("frames.npy")

baseline = 7.5
focal_length = 882.5
max_disparity = 95
max_val = baseline*focal_length
camera_hfov = 69 * np.pi / 180

## first task is to invert the depth map

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

# filters 
# high pass filter
highp = np.array([0,0,-0.003,-0.0066, -0.163,1,0.163,0.066,0.003,0,0])
# low pass filter
lowp = np.array([0,0,1,1,1,1,1,1,1,1,1,0,0])
n = 10
nr = 1
#lowp = np.sinc(np.linspace(-nr, nr-1/n, n))
# differentiator
deriv = np.array([0,1,-1,0])
arx = refined_img
a,b = arx.shape
print(a,b)
w = 640
s = 0
#plt.imshow(refined_img);
#plt.show()
hp = highp[:,np.newaxis]
lp = lowp[:,np.newaxis]
dp = deriv[:,np.newaxis]
conv = lambda x,h: np.apply_along_axis(lambda x: np.convolve(x, h.flatten(), mode='full'),axis=1,arr=x)
cdims = lambda a,x,h: a[:,h.shape[0]-1:h.shape[0]+1+x.shape[1]]
#for i in range(w):
#	arr[i] = np.convolve(arx[:,i],highp)[highp.shape[0]-1:(arx.shape[0]+highp.shape[0]+1)]/(arx.shape[0] + highp.shape[0])	# delay by highp.shape[0]
#	xr[i] = np.convolve(arr[i,:],deriv)[deriv.shape[0]-1:(arr.shape[1]+deriv.shape[0]+1)]/(arr.shape[1]+deriv.shape[0])		# delay by deriv.shape[0]
#	fxx[i] = np.convolve(xr[i,:], lowp)[lowp.shape[0]-1:(xr.shape[1] + lowp.shape[0]+1)]/(xr.shape[1] + lowp.shape[0])		# delay by lowp.shape[0]
#	rx[i] = np.convolve(fxx[i,:], deriv)[deriv.shape[0]-1:(fxx.shape[1]+deriv.shape[0]+1)]/(fxx.shape[1]+deriv.shape[0])		# delay by deriv.shape[0]
#	rx[i] = np.convolve(rx[i,:],lowp)[lowp.shape[0]-1:(rx.shape[1] + lowp.shape[0]+1)]/(rx.shape[1] + lowp.shape[0])		# delay by lowp.shape[0]

arr = cdims(conv(np.transpose(arx),hp),np.transpose(arx), hp)/(arx.shape[0] + highp.shape[0])	# delay by highp.shape[0]
xr = cdims(conv(arr,dp),arr, dp)/(arr.shape[1]+deriv.shape[0])					# delay by deriv.shape[0]
fxx = cdims(conv(xr, lp),xr,lp)/(xr.shape[1] + lowp.shape[0])					# delay by lowp.shape[0]
rx = cdims(conv(fxx, dp),fxx,dp)/(fxx.shape[1]+deriv.shape[0])					# delay by deriv.shape[0]
s = 350
e = 50
d = 300
def obs_detect(fxx, rx,avglen=4):
	fxx_delay = len(deriv)
	rx_delay = len(deriv) * 2 + 1
	xdot_sigma = np.std(fxx[50+fxx_delay:350+fxx_delay])
	xdotdot_sigma = np.std(rx[50+rx_delay:350+rx_delay])
	dot_crossing = np.zeros((d, 2))
	dot_crossing[:,0] = -1
	dot_arr = np.zeros(d) 
	foo = 0
	for i in range(1, d):
		#print(d-i+1)
		if np.sign(np.mean(fxx[s-i:s-i+avglen])) == np.sign(dot_crossing[d - i, 0]) and foo < 1:
			dot_crossing[d - i-1 , 0] = np.mean(fxx[s-i:s-i+avglen])
		else:
			foo = 1

		if abs(np.mean(fxx[s-i:s-i+avglen])) - 6 * xdot_sigma > 0:
			dot_arr[d - i] = 1
	
	e = np.zeros(300)
	counter = len(dot_arr)-1
	while counter != 0:
		if dot_arr[counter] > 0:
			break
		counter-=1
	dot_crossing[0:counter,0] = 0
	e = dot_crossing[:,0]
	e = (e - min(e)) / (max(e) - min(e))
	return e
	
#rect_arr = np.zeros((w,300))


rect_arr = np.array([obs_detect(fxx[i],rx[i],8) for i in range(w)])
free_spaces = np.ones((640,300))
for i in range(w):
	foo = 0;
	hi = 0
	hj = 0
	for j in reversed(range(290,300)):
		free_spaces[i,j] = arr[i,j+49]
	for j in reversed(range(290)):
		if rect_arr[i,j] != 0 and foo != 1:
			free_spaces[i,j] = arr[i,j+49]
		else:
			if foo == 0:
				hi = i
				hj = j+1
			free_spaces[i,0:j] = free_spaces[hi,hj]
			break
			
plt.imshow(np.transpose(free_spaces))
plt.show() 
			
		







