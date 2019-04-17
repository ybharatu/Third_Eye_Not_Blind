#################################################################
# Import Packages
#################################################################
import os
import sys
import numpy as np
import cv2
import matplotlib
matplotlib.use('PS')
import matplotlib.image as mpimg

imgshape0 = 384
imgshape1 = 640
percent = 0.7

leftavgSlope = 0.5
rightavgSlope = -0.5

#leftavgIntercept = lefty2 - (leftavgSlope * leftx2)
#rightavgIntercept = righty2 - (rightavgIntercept * rightx2)
leftavgIntercept = 100
rightavgIntercept = 400

left_line_x1 = int((percent * imgshape0 - leftavgIntercept) / leftavgSlope)
left_line_x2 = int((imgshape0 - leftavgIntercept) / leftavgSlope)

right_line_x1 = int((percent * imgshape0 - rightavgIntercept) / rightavgSlope)
right_line_x2 = int((imgshape0 - rightavgIntercept) / rightavgSlope)



line_img = np.zeros((imgshape0, imgshape1, 3), dtype=np.uint8)

# left lane line
#cv2.line(line_img, (left_line_x1, int(percent * imgshape0)), (left_line_x2, int(imgshape0)), [255,0,0], 10)
# right lane line
#cv2.line(line_img, (right_line_x1, int(percent * imgshape0)), (right_line_x2, int(imgshape0)), [255,255,0], 10)

cv2.line(line_img, (300, 280), (280, 300), [255,255,0], 10)
cv2.line(line_img, (400, 268), (600, 384), [0,255,0], 10)

cv2.line(line_img, (500, 200), (550, 200), [0,255,255], 10)

mpimg.imsave("source_images/my_img.jpg", line_img)