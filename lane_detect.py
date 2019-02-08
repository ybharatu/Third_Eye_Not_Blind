# Adapted from https://github.com/galenballew/SDC-Lane-and-Vehicle-Detection-Tracking

#################################################################
# Import Packages
#################################################################
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import time
from multiprocessing import Process, Value, Array, Lock
import multiprocessing
import os
import sys
import getopt
from ctypes import c_wchar_p

#################################################################
# Lists used for draw_lines
#################################################################
rightSlope, leftSlope, rightIntercept, leftIntercept = [], [], [], []

#################################################################
# Other Global Variables
#################################################################
out_ext = ".png"

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
    yelupper = np.array([50, 255, 255])

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
    return cv2.Canny(grayscale(img), 50, 120)

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
    x = int(img.shape[1])
    y = int(img.shape[0])
    shape = np.array([[int(0), int(y)], [int(x), int(y)], [int(0.55*x), int(0.6*y)], [int(0.45*x), int(0.6*y)]])

    #define a numpy array with the dimensions of img, but comprised of zeros
    mask = np.zeros_like(img)

    #Uses 3 channels or 1 channel for color depending on input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255

    #creates a polygon with the mask color
    cv2.fillPoly(mask, np.int32([shape]), ignore_mask_color)

    #returns the image only where the mask pixels are not zero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

#################################################################
# Function: draw_lines
# Description: Takes in an image and a list of
# (start_pt1, end_pt1, start_pt2, end_pt2). Draws those lines
# onto the given image and returns the image.
#################################################################
def draw_lines(img, lines, thickness=5):
    global rightSlope, leftSlope, rightIntercept, leftIntercept
    rightColor = [0, 255, 0]
    leftColor = [255, 0, 0]

    # this is used to filter out the outlying lines that can affect the average
    # We then use the slope we determined to find the y-intercept of the filtered lines by solving for b in y=mx+b
    for line in lines:
        for x1, y1, x2, y2 in line:
            slope = (y1 - y2) / (x1 - x2)
            if slope > 0.3:
                if x1 > 500:
                    yintercept = y2 - (slope * x2)
                    rightSlope.append(slope)
                    rightIntercept.append(yintercept)
                else:
                    None
            elif slope < -0.3:
                if x1 < 600:
                    yintercept = y2 - (slope * x2)
                    leftSlope.append(slope)
                    leftIntercept.append(yintercept)


                    # We use slicing operators and np.mean() to find the averages of the 30 previous frames
    # This makes the lines more stable, and less likely to shift rapidly
    leftavgSlope = np.mean(leftSlope[-30:])
    leftavgIntercept = np.mean(leftIntercept[-30:])

    rightavgSlope = np.mean(rightSlope[-30:])
    rightavgIntercept = np.mean(rightIntercept[-30:])

    # Here we plot the lines and the shape of the lane using the average slope and intercepts
    try:
        left_line_x1 = int((0.65 * img.shape[0] - leftavgIntercept) / leftavgSlope)
        left_line_x2 = int((img.shape[0] - leftavgIntercept) / leftavgSlope)

        right_line_x1 = int((0.65 * img.shape[0] - rightavgIntercept) / rightavgSlope)
        right_line_x2 = int((img.shape[0] - rightavgIntercept) / rightavgSlope)

        pts = np.array([[left_line_x1, int(0.65 * img.shape[0])], [left_line_x2, int(img.shape[0])],
                        [right_line_x2, int(img.shape[0])], [right_line_x1, int(0.65 * img.shape[0])]], np.int32)
        pts = pts.reshape((-1, 1, 2))
        cv2.fillPoly(img, [pts], (0, 0, 255))

        cv2.line(img, (left_line_x1, int(0.65 * img.shape[0])), (left_line_x2, int(img.shape[0])), leftColor, 10)
        cv2.line(img, (right_line_x1, int(0.65 * img.shape[0])), (right_line_x2, int(img.shape[0])), rightColor, 10)
    except ValueError:
        # I keep getting errors for some reason, so I put this here. Idk if the error still persists.
        pass

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
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    draw_lines(line_img, lines)
    return line_img
    #return lines

#################################################################
# Function: linedetect
# Description: Wrapper file for hough_lines. Can be used to
# modify parameters.
#################################################################
def linedetect(img):
    return hough_lines(img, 1, np.pi/180, 10, 20, 100)

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

#################################################################
# Function: write_images
# Description: Dequeues an image from the buffer and writes it
# as a file. Can be modified to not write images
#################################################################
def write_images(out_buf, filename):
    imgs = 0
    while imgs is not 100:
        while out_buf.empty():
            pass

        processed = out_buf.get()
        mpimg.imsave(filename + "_processed" + out_ext, processed)
        print("Written Image " + str(imgs))
        imgs += 1

#################################################################
# Function: get_images
# Description: Enqueues images onto the buffer.
#################################################################
def get_images(img_buf,filename):
    imgs = 0
    print("FILENAME in get_images: " + filename)
    print("FILENAME: " + str(len(filename)))
    while imgs is not 100:
        image = mpimg.imread(filename)
        while img_buf.full():
            pass

        img_buf.put(image)
        print("Image in input buffer " + str(imgs))
        imgs += 1

#################################################################
# Function: processImage
# Description: Dequeues an image from the input buffer.
# Processes the image and then Enqueues it onto the output
# buffer. Can be modified to include timing information
#################################################################
def processImage(img_buf, out_buf):
    imgs = 0
    while imgs is not 100:
        while img_buf.empty():
            pass
        image = img_buf.get()
        #start = time.time()
        interest = roi(image)
        #print("Region of Interest Time: " + str(time.time() - start) + " sec")
        #start = time.time()
        filterimg = color_filter(interest)
        #print("Color Filter Time: " + str(time.time() - start) + " sec")
        #start = time.time()
        canny = cv2.Canny(grayscale(filterimg), 50, 120)
        #print("Canny Edge + Grayscale Time: " + str(time.time() - start) + " sec")
        #start = time.time()
        myline = hough_lines(canny, 1, np.pi / 180, 10, 20, 5)
        #print("Hough Line Transform Time: " + str(time.time() - start) + " sec")
        weighted_img = cv2.addWeighted(myline, 1, image, 0.8, 0)

        print("Finish Image")
        while out_buf.full():
            pass
        out_buf.put(weighted_img)
        print("Image in Output Buffer " + str(imgs) )
        imgs += 1

    #return weighted_img
    #return myline

#################################################################
# Function: Main Function
# Note: if __name__ == '__main__' guard is necessary in order to
# use processes
#################################################################
def main(argv):
    #global filename
    lock = Lock()
    try:
        opts, args = getopt.getopt(argv, "h:i:", ["image="])
    except getopt.GetoptError:
        print('lane_detect.py -i <image_filename> ')
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print('lane_detect.py -i <image_filename>')
            sys.exit()
        elif opt in ("-i", "--image"):
            # filename_value = Value(c_wchar_p, arg.strip())
            # filename = filename_value.value
            filename = Array(c_wchar_p, ('',''),lock=lock)
            filename[0] = arg.strip()
            print("FILENAME: " + str(type(filename)))
            print("FILENAME: " + str(len(filename)))

    print("FILENAME: " + filename[0])
    start = time.time()

    img_buf = multiprocessing.Queue()
    out_buf = multiprocessing.Queue()



    # get_images(img_buf)
    # processImage(img_buf,out_buf)
    # write_images(out_buf)
    img_opening_process = Process(target=get_images, args=(img_buf,filename[0]))
    img_processing_process = Process(target=processImage, args=(img_buf, out_buf))
    img_writing_process = Process(target=write_images, args=(img_buf,filename[0]))


    img_opening_process.start()
    img_processing_process.start()
    img_writing_process.start()

    img_opening_process.join()
    img_processing_process.join()
    img_writing_process.join()

    end = time.time()
    print("Total Time: " + str(end - start) + " sec")

if __name__ == "__main__":
   main(sys.argv[1:])