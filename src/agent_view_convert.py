#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

raw_img = np.load("frames.npy")

baseline = 7.5
focal_length = 882.5
max_disparity = 95
max_val = baseline * focal_length
camera_hfov = 69 * np.pi / 180

## first task is to invert the depth map

# get normalized disparity map to depth space
depth_img = (baseline * focal_length) / (
    (raw_img * (baseline * focal_length) + 1)
    / (baseline * focal_length)
    * max_disparity
    + 1
)

# normalize depth space
min_px = np.min(np.min(depth_img))
max_px = np.max(np.max(depth_img))
norm_depth_img = (depth_img - min_px) / (max_px - min_px)

# invert and shift
norm_depth_img = norm_depth_img * -1 + 1

# convert back to depth space
refined_img = norm_depth_img * (max_px - min_px)

# convert to agent coordinates
lens_center = raw_img.shape[1] * 0.45
max_r = raw_img.shape[0]
a, b = raw_img.shape
mpd = lambda r, theta: r * np.tan(theta / 2)

# coordinate transform
rect_arr = np.zeros((a, b, 3))
for i in range(a):
    for j in range(b):
        r = refined_img[i, j]
        theta = (
            (j - lens_center)
            / abs(mpd(max_r - i, camera_hfov) - lens_center)
            * camera_hfov
            / 2
        )
        if abs(theta) > camera_hfov / 2:
            rect_arr[i, j, :] = np.nan
        else:
            rect_arr[i, j, 0] = r * np.sin(theta)
            rect_arr[i, j, 1] = r * np.cos(theta)
            rect_arr[i, j, 2] = max_r - i
# display
fig = plt.figure()
ax = fig.add_subplot(projection="3d")
colors = np.linspace(1, 1000, 120164)
ax.scatter(rect_arr[:, :, 0], rect_arr[:, :, 1], rect_arr[:, :, 2], c=colors, s=1)
plt.show()
