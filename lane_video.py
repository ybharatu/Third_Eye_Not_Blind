#################################################################
# Library of video processing functions
#################################################################

#################################################################
# Import Packages
#################################################################
import os
import sys
import numpy as np
import cv2
from common import *
from lane_cv import *
import time
import subprocess
import matplotlib
matplotlib.use('PS')
import matplotlib.image as mpimg
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO!  This is probably because you need superuser privileges.  You can achieve this by using 'sudo' to run your script")
except ImportError:
    print("RPi is not installed. It can only be installed on linux enviornments")

#################################################################
# Function: handle_images
# Description: Enqueues images onto the input buffer from an
# input stream. Outputs appropriate values using images fom the
# output buffer.
#################################################################
def handle_images(input_buffers, output_buffers, vid, filename, live, im, save):

    curr_in_buffer = 0
    curr_out_buffer = 0
    # input_buffers = [input_img_1, input_img_2]
    # output_buffers = [output_img_1, output_img_2]
    in_imgs = 0
    out_imgs = 0
    left_drift_cnt = 0
    right_drift_cnt = 0
    vidcap = cv2.VideoCapture(filename)
    #################################################################
    # Code to handle getting images and placing them into buffer.
    # Could be either from a video (indicated by vid = True) or an
    # image (indicated by vid = False). If an image is selected, the
    # same image is processed NUM_FRAMES times in order to provide
    # meaningful timing information
    #################################################################
    while(in_imgs < NUM_FRAMES or out_imgs < NUM_FRAMES):
        #################################################################
        # Code to handle getting images from a source (either a video
        # or an image
        #################################################################
        if in_imgs != NUM_FRAMES:
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
                #input_buffers[curr_in_buffer].put(image)

                in_imgs += 1
                curr_in_buffer = (curr_in_buffer + 1) % NUM_WORKERS
            else:
                print("No more images")

        #################################################################
        # Consuming elements in the output buffer and returning drift
        # values
        #################################################################
        if(output_buffers[curr_out_buffer].empty()):
            #print("output buffer empty")
            continue

        #################################################################
        # Incraments Appropriate drift counter Key: left drift = -1,
        # right drift = 1, no drift = 0
        #################################################################
        drift_value = output_buffers[curr_out_buffer].get()
        curr_out_buffer = (curr_out_buffer + 1) % NUM_WORKERS

        #print("img " + str(out_imgs) + " taken off output buffer")
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

        #################################################################
        # Note: Need to tell micro that the system is drifting
        #################################################################
        if(left_drift_cnt >= num_drifts_thresh):
            print("Drifting Left!! img"+ str(out_imgs))
        elif(right_drift_cnt >= num_drifts_thresh):
            print("Drifting Right!! img" + str(out_imgs))
        else:
            #print("Not Drifting " + "left_cnt: " + str(left_drift_cnt) + " right_cnt: " + str(right_drift_cnt))
            print("Not Drifting    img" + str(out_imgs))
        out_imgs += 1


#################################################################
# Function: processImage
# Description: Dequeues an image from the input buffer.
# Processes the image and then Enqueues it onto the output
# buffer. Can be modified to include timing information
#################################################################
def processImage(img_buf, out_buf, save, which_worker):

    imgs = 0 # Counter to keep track of number of images
    #################################################################
    # Iterate until the process finished its share of images
    # (specified by NUM_FRAMES / NUM_WORKERS)
    #################################################################
    while imgs is not int(NUM_FRAMES / NUM_WORKERS):
        #################################################################
        # Waits until the Image Handler process puts an image into the
        # input buffer.
        #################################################################
        while img_buf.empty():
            pass

        #################################################################
        # Dequeues an image from the input buffer
        #################################################################
        image = img_buf.get()

        #################################################################
        # Crops region of interest on image based on a polygon
        #################################################################
        #interest = roi(image)

        #################################################################
        # Applies a color filter to the cropped image
        #################################################################
        #filterimg = color_filter(interest)
        filterimg = color_filter(image)

        #################################################################
        # Applies Canny Edge detection on the grayscale image
        #################################################################
        #canny = cv2.Canny(grayscale(filterimg), 50, 120)
        canny_img = canny(filterimg)
        
        #################################################################
        # Crops region of interest on image based on a polygon
        #################################################################
        interest = roi(canny_img)
    

        #################################################################
        # If save flag is asserted, lane lines are drawn over the image.
        # Lines specifying the vehicle and the center of the lanes are
        # also drawn. The resulting image is then saved to a file
        #################################################################
        if save:
            #myline = hough_lines(interest, 1, np.pi / 180, 10, 20, 5)
            myline = linedetect(interest)
            weighted_img = cv2.addWeighted(myline, 1, image, 0.8, 0)
            canny_rgb = cv2.cvtColor(interest, cv2.COLOR_GRAY2RGB)
            weighted_img_2 = cv2.addWeighted(weighted_img, 1, canny_rgb, 0.8, 0)
            #print("cannyrgb.shape = " + str(canny_rgb.shape))
            mpimg.imsave("processed_images/vidimg_"+ str(imgs) + "_worker_" + str(which_worker)+out_ext, weighted_img_2)

        #################################################################
        # Get current drifting value based on image
        #################################################################
        dist_off = get_drift_value(interest, 1, np.pi / 180, 10, 20, 5)

        #################################################################
        # Wait until output buffer is empty
        #################################################################
        while out_buf.full():
            pass

        #################################################################
        # Queues the offset value into an output buffer
        #################################################################
        out_buf.put(dist_off)

        #################################################################
        # Increments image count
        #################################################################
        imgs += 1
