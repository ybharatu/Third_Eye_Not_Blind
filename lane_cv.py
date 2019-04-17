#################################################################
# Library of functions helpful for lane detection
#################################################################

#################################################################
# Import Packages
#################################################################
import os
import sys
import numpy as np
import cv2
import math
from common import *

#################################################################
# Function: color_filter
# Description: Takes in an image and masks white and yellow
# colors (Lane markings are generally white or yellow). Returns
# an image.
#################################################################
def color_filter(image):
    # convert to HLS to mask based on HLS
    hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
    lower = np.array([0, 190, 0])
    upper = np.array([255, 255, 255])

    yellower = np.array([10, 0, 90])
    yelupper = np.array([150, 255, 255])

    yellowmask = cv2.inRange(hls, yellower, yelupper)
    whitemask = cv2.inRange(hls, lower, upper)

    mask = cv2.bitwise_or(yellowmask, whitemask)
    masked = cv2.bitwise_and(image, image, mask=mask)

    return masked


#################################################################
# Function: grayscale
# Description: Takes in an image and transforms it into
# grayscale. Returns an image.
#################################################################
def grayscale(img):
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)


#################################################################
# Function: canny
# Description: Takes in an image and applies canny edge
# detection. Returns an image.
#################################################################
def canny(img):
    sigma = 0.4
    grayimg = grayscale(img)
    v = np.median(grayimg)
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    if (upper == 0 or lower == 0):
        upper = 120
        lower = 50

    #return cv2.Canny(grayscale(img), 50, 120)
    #print("lower = " + str(lower) + " upper = " + str(upper))
    return cv2.Canny(grayimg, lower, upper)


#################################################################
# Function: gaussian_blur
# Description: Takes an image and kernel size and applies a
# gaussian noise kernel. Returns an image.
#################################################################
def gaussian_blur(img, kernel_size):
    """Applies a Gaussian Noise kernel"""
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)


#################################################################
# Function: roi
# Description: Takes an image crops a region of interest. Returns
# masked image.
#################################################################
def roi(img):
    x = int(img.shape[1])  # width
    y = int(img.shape[0])  # height
    topline_percent_x = 0.3  # proportion of base line you want top line of trapazoid to be  /___\
    topline_percent_y = 0.80  # how tall you want the trapazoid to be
    leftover = (1 - topline_percent_x) * x
    leftpoint_x = leftover / 2
    rightpoint_x = leftpoint_x + (topline_percent_x) * x
    point_y = topline_percent_y * y

    """
    print("point1: (0," + str(y) + ")")
    print("point2: (" + str(x) + "," + str(y) + ")")
    print("point3: (" + str(x) + "," + str(y) + ")")
    print("point4: (0," + str(y) + ")")
    """
    shape = np.array(
        [[int(0), int(y)], [int(x), int(y)], [int(rightpoint_x), int(point_y)], [int(leftpoint_x), int(point_y)]])

    # define a numpy array with the dimensions of img, but comprised of zeros
    mask = np.zeros_like(img)

    # Uses 3 channels or 1 channel for color depending on input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255

    # creates a polygon with the mask color
    cv2.fillPoly(mask, np.int32([shape]), ignore_mask_color)

    # returns the image only where the mask pixels are not zero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image



#################################################################
# Function: roi_return_array
# Description: Takes an image crops a region of interest. Returns
# masked image.
#################################################################
def roi_return_array(img):
    x = int(img.shape[1])  # width
    y = int(img.shape[0])  # height
    topline_percent_x = 0.3  # proportion of base line you want top line of trapazoid to be  /___\
    topline_percent_y = 0.80  # how tall you want the trapazoid to be
    leftover = (1 - topline_percent_x) * x
    leftpoint_x = leftover / 2
    rightpoint_x = leftpoint_x + (topline_percent_x) * x
    point_y = topline_percent_y * y

    """
    print("point1: (0," + str(y) + ")")
    print("point2: (" + str(x) + "," + str(y) + ")")
    print("point3: (" + str(x) + "," + str(y) + ")")
    print("point4: (0," + str(y) + ")")
    """
    shape = np.array(
        [[int(0), int(y)], [int(x), int(y)], [int(rightpoint_x), int(point_y)], [int(leftpoint_x), int(point_y)]])

    return shape





#################################################################
# Function: resize_n_crop
# Description: Takes in an image and reduces the resolution by
# half, then crops the bottom 40% of the image and returns the
# resulting image
#################################################################
def resize_n_crop(image):
    # cut image and lower resolution
    res_percent = 0.5  # reduce resolution by this percent
    new_width = int(image.shape[1] * res_percent)
    r = new_width / image.shape[1]
    dim = (new_width, int(image.shape[0] * r))

    #resized = cv2.resize(image, dim)
    resized = image
    # crop image
    croptop_percent_y = 0  # crop this percent of image from the top half
    cropbottom_percent_y = 0.8 #crop this percent of image from the bottom half
    cropped = resized[int(resized.shape[0] * croptop_percent_y):int(resized.shape[0] * cropbottom_percent_y), 0:resized.shape[1]]  # img[y:y+h, x:x+w]
    return cropped

#################################################################
# Function: findLongestLineIdx
# Description: Takes in a list and returns the index of the
# max element
#################################################################
def findLongestLineIdx(linelist):
    maxlen = 0
    bestlnidx = None
    for i, lnlen in enumerate(linelist):
        if lnlen > maxlen:
            maxlen = lnlen
            bestlnidx = i
    return bestlnidx

#################################################################
# Function: draw_lines
# Description: Takes in an image and a list of
# (start_pt1, end_pt1, start_pt2, end_pt2). Draws those lines
# onto the given image and returns the image.
#################################################################
def draw_lines(img, lines, thickness=5):
    global rightSlopeDL, leftSlopeDL, rightInterceptDL, leftInterceptDL
    global num_frame_persist, rightTTL, leftTTL
    #single frame variables
    rightSlopeSF = []
    leftSlopeSF = []
    rightInterceptSF = []
    leftInterceptSF = []
    rightLinesSF = []
    leftLinesSF = []
    ################
    img_mid_point = img.shape[1] / 2
    # num_frame_persist = 5
    rightColor = [0, 255, 0]
    leftColor = [255, 0, 0]
    middleStatColor = [255, 255, 0]
    middleDynColor = [255, 255, 255]
    houghcolor = [255, 0, 255]
    roiColor = [0,0,0]
    drift_threshold = 50
    img_mid_point = img.shape[1] / 2
    #print("img_mid_point = " + str(img_mid_point))
    lines_height_percent = 0.80
    foundleft = False
    foundright = False

    #print("img.shape[0] = " + str(img.shape[0]) + "img.shape[1] = " + str(img.shape[1]))
    # this is used to filter out the outlying lines that can affect the average
    # We then use the slope we determined to find the y-intercept of the filtered lines by solving for b in y=mx+b
    for line in lines:
        for x1, y1, x2, y2 in line:
            # print("x1: " + str(x1) + "x2: " + str(x2) + "y1: " + str(y1) + "y2: " + str(y2))
            denom = x1 - x2
            if denom == 0:
                # print("divided by zero in draw_lines")
                continue
            slope = (y1 - y2) / denom
            # print("slope ===== " + str(slope))

            #Detect right line!!!
            if slope > 0.4 and slope < 2 and x1 > img_mid_point:
                yintercept = y2 - (slope * x2)
                # extended_line_x1 = int((lines_height_percent * img.shape[0] - yintercept) / slope)
                # if extended_line_x1 > img_mid_point+5:
                # print("right slope = " + str(slope) + " x1 = " + str(x1))
                # cv2.line(img, (x1, y1), (x2, y2), houghcolor, 5)
                # if(len(rightSlopeDL) > 0):
                #     rightSlopeDL.pop(0)
                #     rightInterceptDL.pop(0)
                # rightSlopeDL.append(slope)
                # rightInterceptDL.append(yintercept)
                linelen = math.sqrt(math.pow((x2 - x1), 2) + math.pow((y2 - y1), 2))
                rightSlopeSF.append(slope)
                rightInterceptSF.append(yintercept)
                rightLinesSF.append(linelen)
                foundright = True

            #Detect left line!!!
            elif slope < -0.4 and slope > -2 and x1 < img_mid_point:
                yintercept = y2 - (slope * x2)
                # extended_line_x1 = int((lines_height_percent * img.shape[0] - yintercept) / slope)
                # if extended_line_x1 < img_mid_point-5:
                # print("left slope = " + str(slope) + " x1 = " + str(x1))
                # cv2.line(img, (x1, y1), (x2, y2), houghcolor, 5)
                # if (len(leftSlopeDL) > 0):
                #     leftSlopeDL.pop(0)
                #     leftInterceptDL.pop(0)
                # leftSlopeDL.append(slope)
                # leftInterceptDL.append(yintercept)
                linelen = math.sqrt(math.pow((x2 - x1), 2) + math.pow((y2 - y1), 2))
                leftSlopeSF.append(slope)
                leftInterceptSF.append(yintercept)
                leftLinesSF.append(linelen)
                foundleft = True



    bestRightIdx = findLongestLineIdx(rightLinesSF)
    bestLeftIdx = findLongestLineIdx(leftLinesSF)


    if bestLeftIdx != None:
        #left lines were found
        # print("chosen left line slope = " + str(leftSlopeSF[bestLeftIdx]))
        leftSlopeDL.append(leftSlopeSF[bestLeftIdx])
        leftInterceptDL.append(leftInterceptSF[bestLeftIdx])
    if bestRightIdx != None:
        #right lines were found
        # print("chosen right line slope = " + str(rightSlopeSF[bestRightIdx]))
        rightSlopeDL.append(rightSlopeSF[bestRightIdx])
        rightInterceptDL.append(rightInterceptSF[bestRightIdx])

    lenright = len(rightSlopeDL)
    lenleft = len(leftSlopeDL)
    if lenright > num_frame_persist:
        rightSlopeDL.pop(0)
        rightInterceptDL.pop(0)
    if lenleft > num_frame_persist:
        leftSlopeDL.pop(0)
        leftInterceptDL.pop(0)
    if lenright == 0 or lenleft == 0:
        # no lines!!!
        return




    # print("leftSlope = " + leftSlope + "  rightSlope = " + rightSlope)
    # print("------------")
    # print(leftSlopeDL)
    # print(rightSlopeDL)
    # print("------------")

        # We use slicing operators and np.mean() to find the averages of the 30 previous frames
    # This makes the lines more stable, and less likely to shift rapidly
    # leftavgSlope = np.mean(leftSlopeDL[-num_frame_persist:])
    # leftavgIntercept = np.mean(leftInterceptDL[-num_frame_persist:])
    #
    # rightavgSlope = np.mean(rightSlopeDL[-num_frame_persist:])
    # rightavgIntercept = np.mean(rightInterceptDL[-num_frame_persist:])
    #########################
    # leftavgSlope = np.mean(leftSlopeDL)
    # leftavgIntercept = np.mean(leftInterceptDL)
    #
    # rightavgSlope = np.mean(rightSlopeDL)
    # rightavgIntercept = np.mean(rightInterceptDL)
    #########################
    leftavgSlope = leftSlopeDL[-1]
    leftavgIntercept = leftInterceptDL[-1]

    rightavgSlope = rightSlopeDL[-1]
    rightavgIntercept = rightInterceptDL[-1]


    # Here we plot the lines and the shape of the lane using the average slope and intercepts
    # try:
    left_line_x1 = int((lines_height_percent * img.shape[0] - leftavgIntercept) / leftavgSlope)
    left_line_x2 = int((img.shape[0] - leftavgIntercept) / leftavgSlope)

    right_line_x1 = int((lines_height_percent * img.shape[0] - rightavgIntercept) / rightavgSlope)
    right_line_x2 = int((img.shape[0] - rightavgIntercept) / rightavgSlope)

    lanewidth = right_line_x1 - left_line_x1
    # print("lane width ==== " + str(lanewidth))
    if lanewidth < 70:
        # print("lane width ==== " + str(lanewidth))
        rightSlopeDL.pop(-1)
        rightInterceptDL.pop(-1)
        leftSlopeDL.pop(-1)
        leftInterceptDL.pop(-1)
        if len(rightSlopeDL) == 0 or len(leftSlopeDL) == 0:
            # no lines!!!
            return
        ############ Recalculate everything since we had to throw out that data point
        leftavgSlope = leftSlopeDL[-1]
        leftavgIntercept = leftInterceptDL[-1]
        rightavgSlope = rightSlopeDL[-1]
        rightavgIntercept = rightInterceptDL[-1]
        left_line_x1 = int((lines_height_percent * img.shape[0] - leftavgIntercept) / leftavgSlope)
        left_line_x2 = int((img.shape[0] - leftavgIntercept) / leftavgSlope)
        right_line_x1 = int((lines_height_percent * img.shape[0] - rightavgIntercept) / rightavgSlope)
        right_line_x2 = int((img.shape[0] - rightavgIntercept) / rightavgSlope)
        ##############


    midstat_line_x1 = int((img.shape[1] / 2))
    midstat_line_x2 = int((img.shape[1] / 2))

    middyn_line_x1 = int(((left_line_x1 + right_line_x1) / 2))
    middyn_line_x2 = middyn_line_x1

    pts = np.array([[left_line_x1, int(lines_height_percent * img.shape[0])], [left_line_x2, int(img.shape[0])],
                    [right_line_x2, int(img.shape[0])], [right_line_x1, int(lines_height_percent * img.shape[0])]], np.int32)
    #pts = pts.resha                              pe((-1, 1, 2))
    # cv2.fillPoly(img, [pts], (0, 0, 255))


    # make roi polygon
    shape = roi_return_array(img)
    # cv2.fillPoly(img, np.int32([shape]), (0,0,255))

    leftshape = np.array([[int(50), int(50)], [int(100), int(50)], [int(100), int(100)], [int(50), int(100)]])
    rightshape = np.array([[int(img.shape[1] - 100), int(50)], [int(img.shape[1] - 50), int(50)], [int(img.shape[1] - 50), int(100)], [int(img.shape[1] - 100), int(100)]])

    # this is just for the find_position_in_lane function and it is here temporarily
    center_line_x = int((img.shape[1] / 2))
    mid_lane_x = int(((left_line_x1 + right_line_x1) / 2))
    off_center_dist = center_line_x - mid_lane_x
    if off_center_dist > drift_threshold:
        print("!!!Drifting right!!!")  # drifting right
        cv2.fillPoly(img, np.int32([rightshape]), (255,0,0))
    elif off_center_dist < (-1) * drift_threshold:
        print("!!!Drifting left!!!")  # drifting left
        cv2.fillPoly(img, np.int32([leftshape]), (255, 0, 0))
    else:
        pass #not drifting

    # left lane line
    cv2.line(img, (left_line_x1, int(lines_height_percent * img.shape[0])), (left_line_x2, int(img.shape[0])), leftColor, 10)
    # right lane line
    cv2.line(img, (right_line_x1, int(lines_height_percent * img.shape[0])), (right_line_x2, int(img.shape[0])), rightColor, 10)
    # middle of frame line (yellow)
    cv2.line(img, (midstat_line_x1, int(lines_height_percent * img.shape[0])), (midstat_line_x2, int(img.shape[0])),
             middleStatColor, 10)
    # middle of lane line (white)
    cv2.line(img, (middyn_line_x1, int(lines_height_percent * img.shape[0])), (middyn_line_x2, int(img.shape[0])), middleDynColor,
             10)



    # except ValueError:
    #     # I keep getting errors for some reason, so I put this here. Idk if the error still persists.
    #     pass


#################################################################
# Function: find_position_in_lane
# Description: Takes in an image and a list of
# (start_pt1, end_pt1, start_pt2, end_pt2). Return difference
# between center of image and center of lane lines
#################################################################
def find_position_in_lines(img, lines):
    global rightSlope, leftSlope, rightIntercept, leftIntercept
    drift_threshold = 45
    lines_height_percent = 0.75
    img_mid_point = img.shape[1] / 2


    # this is used to filter out the outlying lines that can affect the average
    # We then use the slope we determined to find the y-intercept of the filtered lines by solving for b in y=mx+b
    if(lines is None):
        print("empty lines from hough lines")
        return 3
    for line in lines:
        for x1, y1, x2, y2 in line:
            denom = x1 - x2
            if denom == 0:
                # print("divided by zero in find_pos_in_lines")
                return 3
            slope = (y1 - y2) / denom

            # print("slope = " + str(slope))
            if slope > 0.5:
                if x1 > img_mid_point:
                    yintercept = y2 - (slope * x2)
                    rightSlope.append(slope)
                    rightIntercept.append(yintercept)
                else:
                    None
            elif slope < -0.5:
                if x1 < img_mid_point:
                    yintercept = y2 - (slope * x2)
                    leftSlope.append(slope)
                    leftIntercept.append(yintercept)

    if len(leftSlope) == 0:
        print("not enough left slope lines")
        print("!!!!!!!!!!!!!!!!!!!!!!!!!")
        return 3
    elif len(rightSlope) == 0:
        print("not enough right slope lines")
        print("!!!!!!!!!!!!!!!!!!!!!!!!!")
        return 3

    try:
        # We use slicing operators and np.mean() to find the averages of the 30 previous frames
        # This makes the lines more stable, and less likely to shift rapidly
        leftavgSlope = np.mean(leftSlope[-30:])
        leftavgIntercept = np.mean(leftIntercept[-30:])

        rightavgSlope = np.mean(rightSlope[-30:])
        rightavgIntercept = np.mean(rightIntercept[-30:])

        left_line_x1 = int((lines_height_percent * img.shape[0] - leftavgIntercept) / leftavgSlope)

        right_line_x1 = int((lines_height_percent * img.shape[0] - rightavgIntercept) / rightavgSlope)

        center_line_x = int((img.shape[1] / 2))

        mid_lane_x = int(((left_line_x1 + right_line_x1) / 2))

        off_center_dist = center_line_x - mid_lane_x
        # print("off center dist: " + str(off_center_dist))
        # print("offset: " + str(off_center_dist))
        # print("Off Center Distance: " + str(off_center_dist))
    except:
        return 3

    if off_center_dist > drift_threshold:
        return 1  # drifting right
    elif off_center_dist < (-1) * drift_threshold:
        return -1  # drifting left
    else:
        return 0


#################################################################
# Function: hough_lines
# Description: Takes in the parameters of OpenCV HoughLinesP
# function. Can be modified to draw/not draw the lines on the
# image. Returns either line with image or list of
# (start_pt1, end_pt1, start_pt2, end_pt2).
#################################################################
def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    """
    `img` should be the output of a Canny transform.
    """
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len,
                            maxLineGap=max_line_gap)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    if (lines is not None):
        draw_lines(line_img, lines)
    return line_img


#################################################################
# Function: get_drift_value
# Description: Takes in the parameters of OpenCV HoughLinesP
# function. Returns if drifting left, drifting right, not
# drifting, or if no lines can be found
#################################################################
def get_drift_value(img, rho, theta, threshold, min_line_len, max_line_gap):
    """
    `img` should be the output of a Canny transform.
    """
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len,
                            maxLineGap=max_line_gap)
    return find_position_in_lines(img, lines)


#################################################################
# Function: linedetect
# Description: Wrapper file for hough_lines. Can be used to
# modify parameters.
#################################################################
def linedetect(img):
    return hough_lines(img, 1, np.pi / 180, 30, 20, 10)


#################################################################
# Function: weighted_img
# Description: Wrapper file for OpenCV addWeighted. Can be used
# to modify parameters
#################################################################
def weighted_img(img, initial_img, α=0.8, β=1., λ=0.):
    """
    initial_img * α + img * β + λ
    NOTE: initial_img and img must be the same shape!
    """
    return cv2.addWeighted(initial_img, α, img, β, λ)
