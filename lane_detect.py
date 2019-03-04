# Adapted from https://github.com/galenballew/SDC-Lane-and-Vehicle-Detection-Tracking

#################################################################
# Import Packages
#################################################################
import matplotlib
matplotlib.use('PS')
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import time
from multiprocessing import Process, Value, Array, Lock
import multiprocessing
import asyncio
import os
import sys
import getopt
import timeit
from ctypes import c_wchar_p
from cProfile import Profile
from pstats import Stats
import picamera
import io


NUM_WORKERS = 2
NUM_FRAMES = 210 #should be multiple of 3
#################################################################
# Lists used for draw_lines
#################################################################
rightSlope, leftSlope, rightIntercept, leftIntercept = [], [], [], []

#################################################################
# Other Global Variables
#################################################################
out_ext = ".jpg"
idx = 0



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
# Function: resize_n_crop
# Description: Takes in an image and reduces the resolution by
# half, then crops the bottom 40% of the image and returns the
# resulting image
#################################################################
def resize_n_crop(image):
    # cut image and lower resolution
    res_percent = 0.5 #reduce resolution by this percent
    new_width = int(image.shape[1]*res_percent)
    r = new_width / image.shape[1]
    dim = (new_width, int(image.shape[0] * r))

    resized = cv2.resize(image, dim)
    #crop image
    crop_percent_y = 0.6 #crop this percent of image from the top half
    cropped = resized[int(resized.shape[0] * crop_percent_y):resized.shape[0], 0:resized.shape[1]] #img[y:y+h, x:x+w]
    return cropped


#################################################################
# Function: draw_lines
# Description: Takes in an image and a list of
# (start_pt1, end_pt1, start_pt2, end_pt2). Draws those lines
# onto the given image and returns the image.
#################################################################
def draw_lines(img, lines, thickness=5):
    global rightSlope, leftSlope, rightIntercept, leftIntercept
    img_mid_point = img.shape[1] / 2
    rightColor = [0, 255, 0]
    leftColor = [255, 0, 0]
    middleStatColor = [255, 255, 0]
    middleDynColor = [255, 255, 255]
    drift_threshold = 50
    img_mid_point = img.shape[1] / 2

    # this is used to filter out the outlying lines that can affect the average
    # We then use the slope we determined to find the y-intercept of the filtered lines by solving for b in y=mx+b
    for line in lines:
        for x1, y1, x2, y2 in line:
            slope = (y1 - y2) / (x1 - x2)
            if slope > 0.3:
                if x1 > img_mid_point:
                    yintercept = y2 - (slope * x2)
                    rightSlope.append(slope)
                    rightIntercept.append(yintercept)
                else:
                    None
            elif slope < -0.3:
                if x1 < img_mid_point:
                    yintercept = y2 - (slope * x2)
                    leftSlope.append(slope)
                    leftIntercept.append(yintercept)


    if len(leftSlope) == 0:
        print("not enough left slope lines")
        print("!!!!!!!!!!!!!!!!!!!!!!!!!")
        sys.exit(1)
    elif len(rightSlope) == 0:
        print("not enough right slope lines")
        print("!!!!!!!!!!!!!!!!!!!!!!!!!")
        sys.exit(1)

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

        midstat_line_x1 = int((img.shape[1] / 2))
        midstat_line_x2 = int((img.shape[1] / 2))

        middyn_line_x1 = int(((left_line_x1 + right_line_x1)/ 2))
        middyn_line_x1 = int(((left_line_x1 + right_line_x1) / 2))
        middyn_line_x2 = middyn_line_x1

        pts = np.array([[left_line_x1, int(0.65 * img.shape[0])], [left_line_x2, int(img.shape[0])],
                        [right_line_x2, int(img.shape[0])], [right_line_x1, int(0.65 * img.shape[0])]], np.int32)
        pts = pts.reshape((-1, 1, 2))
        cv2.fillPoly(img, [pts], (0, 0, 255))


        cv2.line(img, (left_line_x1, int(0.65 * img.shape[0])), (left_line_x2, int(img.shape[0])), leftColor, 10)
        cv2.line(img, (right_line_x1, int(0.65 * img.shape[0])), (right_line_x2, int(img.shape[0])), rightColor, 10)
        cv2.line(img, (midstat_line_x1, int(0.65 * img.shape[0])), (midstat_line_x2, int(img.shape[0])),
                 middleStatColor, 10)
        cv2.line(img, (middyn_line_x1, int(0.65 * img.shape[0])), (middyn_line_x2, int(img.shape[0])), middleDynColor,
                 10)

    except ValueError:
        # I keep getting errors for some reason, so I put this here. Idk if the error still persists.
        pass

#################################################################
# Function: find_position_in_lane
# Description: Takes in an image and a list of
# (start_pt1, end_pt1, start_pt2, end_pt2). Return difference
# between center of image and center of lane lines
#################################################################
def find_position_in_lines(img, lines):
    global rightSlope, leftSlope, rightIntercept, leftIntercept
    drift_threshold = 50
    img_mid_point = img.shape[1]/2

    # this is used to filter out the outlying lines that can affect the average
    # We then use the slope we determined to find the y-intercept of the filtered lines by solving for b in y=mx+b
    for line in lines:
        for x1, y1, x2, y2 in line:
            slope = (y1 - y2) / (x1 - x2)
            #print("slope = " + str(slope))
            if slope > 0.2:
                if x1 > img_mid_point:
                    yintercept = y2 - (slope * x2)
                    rightSlope.append(slope)
                    rightIntercept.append(yintercept)
                else:
                    None
            elif slope < -0.2:
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

    # We use slicing operators and np.mean() to find the averages of the 30 previous frames
    # This makes the lines more stable, and less likely to shift rapidly
    leftavgSlope = np.mean(leftSlope[-30:])
    leftavgIntercept = np.mean(leftIntercept[-30:])

    rightavgSlope = np.mean(rightSlope[-30:])
    rightavgIntercept = np.mean(rightIntercept[-30:])

    left_line_x1 = int((0.65 * img.shape[0] - leftavgIntercept) / leftavgSlope)

    right_line_x1 = int((0.65 * img.shape[0] - rightavgIntercept) / rightavgSlope)

    center_line_x = int((img.shape[1] / 2))

    mid_lane_x = int(((left_line_x1 + right_line_x1) / 2))

    off_center_dist = center_line_x - mid_lane_x
    # print("off center dist: " + str(off_center_dist))
    # print("offset: " + str(off_center_dist))
    #print("Off Center Distance: " + str(off_center_dist))
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
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    draw_lines(line_img, lines)
    return line_img

def hough_lines_2(img, rho, theta, threshold, min_line_len, max_line_gap):
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
def write_images(out_buf, foo):
    imgs = 0
    first_time_empty = True
    while imgs is not NUM_FRAMES:
        #spin wait while buffer is empty
        while out_buf.empty():
            if first_time_empty:
                print("out buf is empty")
                first_time_empty = False
            else:
                pass
            #pass
        first_time_empty = True
        processed = out_buf.get()
        mpimg.imsave("processed_images/testimg_"+ str(imgs) + out_ext, processed)
        print("Written Image " + str(imgs))
        imgs += 1
    print("Done writing images")



#################################################################
# Function: get_images
# Description: Enqueues images onto the buffer.
#################################################################
def get_images(img_buf, vid, filename):

    if vid:
        # code to capture video
        vidcap = cv2.VideoCapture(filename)
        success = True
        imgs = 0
        #while success:

        while imgs is not NUM_FRAMES:
            # cv2.imwrite("frame%d.jpg" % count, image)     # save frame as JPEG file
            while img_buf.full():
                pass
            success, image = vidcap.read()
            if success:
                img_buf.put(image)
                #print("Image " + str(imgs) + " in input buffer, PID: " + str(os.getPID()))
                imgs += 1
            else:
                print("No more images")
        print("Total frames processed: " + str(imgs))
    else:
        #code to process 100 images
        imgs = 0
        print("FILENAME in get_images: " + filename)
        print("FILENAME: " + str(len(filename)))
        while imgs is not NUM_FRAMES:
            image = mpimg.imread(filename)
            while img_buf.full():
                pass

            img_buf.put(image)
            imgs += 1

#################################################################
# Function: handle_images
# Description: Enqueues images onto the input buffer from an
# input stream. Outputs appropriate values using images fom the
# output buffer.
#################################################################
def handle_images(input_img_1, input_img_2, output_img_1, output_img_2, vid, filename, live):

    curr_in_buffer = 0
    curr_out_buffer = 0
    input_buffers = [input_img_1, input_img_2]
    output_buffers = [output_img_1, output_img_2]
    in_imgs = 0
    out_imgs = 0
    left_drift_cnt = 0
    right_drift_cnt = 0
    #################################################################
    # Code to handle getting images and placing them into buffer.
    # Could be either from a video (indicated by vid = True) or an
    # image (indicated by vid = False). If an image is selected, the
    # same image is processed NUM_FRAMES times in order to provide
    # meaningful timing information
    #################################################################
    while(in_imgs != NUM_FRAMES or out_imgs != NUM_FRAMES):
        #################################################################
        # Code to handle getting images from a source (either a video
        # or an image
        #################################################################
        if vid and in_imgs != NUM_FRAMES:
            vidcap = cv2.VideoCapture(filename)
            #################################################################
            # Check if current buffer is full and wait till it is not
            #################################################################
            while input_buffers[curr_in_buffer].full():
                pass

            success, image = vidcap.read()
            #################################################################
            # Puts image into buffer and updates current buffer
            #################################################################
            if success:
                # cut image and lower resolution
                smaller_img = resize_n_crop(image)
                input_buffers[curr_in_buffer].put(smaller_img)

                in_imgs += 1
                curr_in_buffer = (curr_in_buffer + 1) % NUM_WORKERS
            else:
                print("No more images")

        elif (live):
            in_imgs = 0
            with picamera.PiCamera() as camera:
                while(in_imgs < NUM_FRAMES):
                    camera.capture_continuous("source_images/image" + str(in_imgs) + out_ext)
                    image = mpimg.imread("source_images/image" + str(in_imgs) + out_ext)
                    smaller_img = resize_n_crop(image)
                    input_buffers[curr_in_buffer].put(smaller_img)
                    in_imgs += 1
                    curr_in_buffer = (curr_in_buffer + 1) % NUM_WORKERS
                    time.sleep(2)

        #################################################################
        # Code to handle single image processing (NUM_FRAMES times for timing)
        #################################################################
        elif(in_imgs != NUM_FRAMES) :
            # code to process NUM_FRAMES images
            imgs = 0
            image = mpimg.imread(filename)
            #################################################################
            # Checks if buffer is full and waits till its not full
            #################################################################
            while input_buffers[curr_in_buffer].full():
                pass

            #################################################################
            # Puts image into buffer and updates current buffer
            #################################################################
            input_buffers[curr_in_buffer].put(image)
            print("Image in input buffer " + str(imgs))
            imgs += 1
            curr_in_buffer = (curr_in_buffer + 1) % NUM_WORKERS


        #################################################################
        # Consuming elements in the output buffer and returning drift
        # values
        #################################################################
        if(output_buffers[curr_out_buffer].empty()):
            continue

        #################################################################
        # Incraments Appropriate drift counter Key: left drift = -1,
        # right drift = 1, no drift = 0
        #################################################################
        drift_value = output_buffers[curr_out_buffer].get()
        curr_out_buffer = (curr_out_buffer + 1) % NUM_WORKERS
        #print("Drift Value from output buffer = " + str(drift_value) + ", PID: " + str(os.getpid()))
        if(drift_value == -1):
            right_drift_cnt = 0
            left_drift_cnt += 1
        elif(drift_value == 1):
            right_drift_cnt += 1
            left_drift_cnt = 0
        elif (drift_value == 0):
            right_drift_cnt = 0
            left_drift_cnt = 0
        elif (drift_value == 3):
            print("Could not detect lines")
        else:
            print("Unexpected Value Obtained in Output buffer")
        out_imgs += 1
        #################################################################
        # Note: Need to tell micro that the system is drifting
        #################################################################
        if(left_drift_cnt >= 3):
            print("Drifting Left!!")
        elif(right_drift_cnt >= 3):
            print("Drifting Right!!")
        else:
            #print("Not Drifting " + "left_cnt: " + str(left_drift_cnt) + " right_cnt: " + str(right_drift_cnt))
            print("Not Drifting")


#################################################################
# Function: processImage
# Description: Dequeues an image from the input buffer.
# Processes the image and then Enqueues it onto the output
# buffer. Can be modified to include timing information
#################################################################
def processImage(img_buf, out_buf):
    imgs = 0
    while imgs is not int(NUM_FRAMES / NUM_WORKERS):
        while img_buf.empty():
            pass
        start = time.time()
        image = img_buf.get()
        #print("get img from buffer time: " + str(time.time()-start))
        #interest = roi(image)
        filterimg = color_filter(image)
        #mpimg.imsave("processed_images/testimg_filtered_"+ str(imgs) + out_ext, filterimg)
        canny = cv2.Canny(grayscale(filterimg), 50, 120)
        #mpimg.imsave("processed_images/testimg_canny_"+ str(imgs) + out_ext, canny)
        #myline = hough_lines(canny, 1, np.pi / 180, 10, 20, 5)
        dist_off = hough_lines_2(canny, 1, np.pi / 180, 10, 20, 5)
        #weighted_img = cv2.addWeighted(myline, 1, image, 0.8, 0)
        #mpimg.imsave("processed_images/testing_"+ str(imgs) + "_" + str(os.getpid())+out_ext, weighted_img)

        while out_buf.full():
            pass
        #print("dist_off: " + str(dist_off))
        out_buf.put(dist_off)


        #print("Image " + str(imgs) + " in Output Buffer, PID: " + str(os.getpid()))
        imgs += 1
        #print("NUM_FRAMES / 3 = " + str(int(NUM_FRAMES / 3)) + " and imgs = " + str(imgs))



#################################################################
# Function: Main Function
# Note: if __name__ == '__main__' guard is necessary in order to
# use processes
#################################################################
def main(argv):
    vid = False
    serial = False
    live = False
    try:
        opts, args = getopt.getopt(argv, "hvsli:", ["help", "video", "serial", "live", "image="])
    except getopt.GetoptError:
        print('lane_detect.py -i <filename> -v')
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print('lane_detect.py -i <image_filename>')
            sys.exit()
        elif opt in ("-i", "--image"):
            filename = arg.strip()
        elif opt in ("-v", "--video"):
            vid = True
        elif opt in ("-s", "--serial"):
            serial = True
        elif opt in ("-l", "--live"):
            live = True

    #print("FILENAME: " + filename)


    input_img_1 = multiprocessing.Queue()
    input_img_2 = multiprocessing.Queue()
    #input_img_3 = multiprocessing.Queue()

    output_img_1 = multiprocessing.Queue()
    output_img_2 = multiprocessing.Queue()
    #output_img_3 = multiprocessing.Queue()

    # img_buf = multiprocessing.Queue()
    # out_buf = multiprocessing.Queue()

    # img_opening_process = Process(target=get_images, args=(img_buf, vid, filename))
    # img_processing_process = Process(target=processImage, args=(img_buf, out_buf))
    # img_writing_process = Process(target=write_images, args=(out_buf, None))

    img_handling_process = Process(target=handle_images, args=(input_img_1, input_img_2,\
                                                            output_img_1, output_img_2, vid, filename, live))
    img_processing_1_process = Process(target=processImage, args=(input_img_1, output_img_1))
    img_processing_2_process = Process(target=processImage, args=(input_img_2, output_img_2))
    #img_processing_3_process = Process(target=processImage, args=(input_img_3, output_img_3))

    prof = Profile()
    prof.enable()

    start = time.time()
    img_handling_process.start()
    img_processing_1_process.start()
    img_processing_2_process.start()
    #img_processing_3_process.start()

    img_handling_process.join()
    print("Finished Process 1")
    img_processing_1_process.join()
    print("Finished Process 2")
    img_processing_2_process.join()
    print("Finished Process 3")
    #img_processing_3_process.join()
    #print("Finished Process 4")


    end = time.time()
    print("Total Time: " + str(end - start) + " sec")
    print("FPS: " + str(NUM_FRAMES/(end - start)) + " sec")

    prof.disable()
    prof.dump_stats('mystats.stats')

    with open('mystats_output.txt', 'wt') as output:
        stats = Stats('mystats.stats', stream=output)
        stats.sort_stats('cumulative', 'time')
        stats.print_stats()

    # img_opening_process.start()
    # img_processing_process.start()
    # img_writing_process.start()

    # img_opening_process.join()
    # print("finished process 1")
    # img_processing_process.join()
    # print("finished process 2")
    # img_writing_process.join()
    # print("finished process 2")


if __name__ == "__main__":
   main(sys.argv[1:])
