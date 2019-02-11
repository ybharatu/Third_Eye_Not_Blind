# Adapted from https://github.com/galenballew/SDC-Lane-and-Vehicle-Detection-Tracking

#importing some useful packages
import matplotlib
matplotlib.use('PS')
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import time
import queue as ImgBuf
from multiprocessing import Process
import multiprocessing
import os
import math
#from moviepy.editor import VideoFileClip
#from IPython.display import HTML

sample_num = 1;
filename = "road_sample_" + sample_num.__str__()
ext = '.png'

def color_filter(image):
    # convert to HLS to mask based on HLS
    hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
    lower = np.array([0, 190, 0])
    upper = np.array([255, 255, 255])

    yellower = np.array([10, 0, 90])
    yelupper = np.array([150, 255, 255])
    #bluelower = np.array([10, 0, 90])
    #blueupper = np.array([50, 255, 255])

    yellowmask = cv2.inRange(hls, yellower, yelupper)
    whitemask = cv2.inRange(hls, lower, upper)
    #bluemask = cv2.inRange(hls, bluelower, blueupper)

    mask = cv2.bitwise_or(yellowmask, whitemask)
    masked = cv2.bitwise_and(image, image, mask=mask)

    return masked

def grayscale(img):
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

def canny(img):
    return cv2.Canny(grayscale(img), 50, 120)

def gaussian_blur(img, kernel_size):
    """Applies a Gaussian Noise kernel"""
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

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


rightSlope, leftSlope, rightIntercept, leftIntercept = [], [], [], []


def draw_lines(img, lines, imgID):
    global rightSlope, leftSlope, rightIntercept, leftIntercept
    rightColor = [0, 255, 0]
    leftColor = [255, 0, 0]


    # this is used to filter out the outlying lines that can affect the average
    # We then use the slope we determined to find the y-intercept of the filtered lines by solving for b in y=mx+b
    for line in lines:
        for x1, y1, x2, y2 in line:
            slope = (y1 - y2) / (x1 - x2)
            print("img" + str(imgID) + " x1: " + str(x1) + " x2: " + str(x2) + " y1: " + str(y1) + " y2: " + str(y2) + " slope: " + str(slope) + " yint: " + str(y2 - (slope * x2)))
            if slope > 0.2:
                #if x1 > 500:
                yintercept = y2 - (slope * x2)
                rightSlope.append(slope)
                rightIntercept.append(yintercept)
                #else:
                #    None
            elif slope < -0.2:
                #if x1 < 600:
                yintercept = y2 - (slope * x2)
                leftSlope.append(slope)
                leftIntercept.append(yintercept)
    #if (len(leftSlope) is 0) and len(rightSlope):
    #    yintercept = 820
    #    leftSlope.append(-0.6)
    #    leftIntercept.append(yintercept)
    try:
                    # We use slicing operators and np.mean() to find the averages of the 30 previous frames
    # This makes the lines more stable, and less likely to shift rapidly
        leftavgSlope = np.mean(leftSlope[-30:])
        leftavgIntercept = np.mean(leftIntercept[-30:])

        rightavgSlope = np.mean(rightSlope[-30:])
        rightavgIntercept = np.mean(rightIntercept[-30:])
        print("rightavgintercept: " + str(rightavgIntercept))

    # Here we plot the lines and the shape of the lane using the average slope and intercepts
    #try:
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

def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap, imgID):
    """
    `img` should be the output of a Canny transform.
    """
    #lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    lines = cv2.HoughLinesP(image=img, lines=np.array([]), rho=rho, theta=theta, threshold=threshold, minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    draw_lines(line_img, lines, imgID=imgID)
    return line_img
    #return lines

def linedetect(img):
    #return hough_lines(img, 1, np.pi/180, 10, 20, 100)
    return hough_lines(img, 1, np.pi/180, 10, 10, 200)


# Python 3 has support for cool math symbols.

def weighted_img(img, initial_img, α=0.8, β=1., λ=0.):
    """
    initial_img * α + img * β + λ
    NOTE: initial_img and img must be the same shape!
    """
    return cv2.addWeighted(initial_img, α, img, β, λ)

def weightSum(input_set):
    img = list(input_set)
    return cv2.addWeighted(img[0], 1, img[1], 0.8, 0)

def write_images(out_buf):
    imgs = 0
    while (not end_of_vid) or (not img_buf.empty()):
        while out_buf.empty():
            pass

        processed = out_buf.get()
        mpimg.imsave("test_images/annotaded_" + str(imgs) + ext, processed)
        print("Written Image - count: " + str(imgs))
        imgs += 1
    print("done writing all " + str(imgs) +"images")

end_of_vid = False
def get_images(img_buf):
    """
    imgs = 0
    while imgs is not 100:
        image = mpimg.imread(filename + ext)
        while img_buf.full():
            pass

        img_buf.put(image)
        print("Image in input buffer")
        imgs += 1
    """
    # code to capture video

    #vidcap = cv2.VideoCapture('challenge.mp4')
    vidcap = cv2.VideoCapture('challenge.mp4')
    success = True
    count = 0
    while success:
        # cv2.imwrite("frame%d.jpg" % count, image)     # save frame as JPEG file
        while img_buf.full():
            pass
        success, image = vidcap.read()
        if success:
            img_buf.put(image)
            print("Image in input buffer - PID: "+ str(os.getpid()))
            count += 1
        else:
            print("No more images")
            end_of_vid = True
            break # should not need this, why doesnt this loop exit??
    #return end_of_vid
    print("Total frames processed: " + str(count))

def processImage(img_buf, out_buf):
    imgs = 0
    #while imgs is not 100:
    while (not end_of_vid) or (not img_buf.empty()):
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
        canny = cv2.Canny(grayscale(filterimg), 50, 150)
        #print("Canny Edge + Grayscale Time: " + str(time.time() - start) + " sec")
        #start = time.time()
        myline = hough_lines(canny, 1, np.pi / 180, 10, 20, 5, imgID = imgs)
        #print("Hough Line Transform Time: " + str(time.time() - start) + " sec")
        #weighted_img = cv2.addWeighted(myline, 1, image, 0.8, 0)
        weighted_img = cv2.addWeighted(myline, 1, image, 0.8, 0)

        print("Finish Image - PID: " + str(os.getpid()))
        while out_buf.full():
            pass
        #out_buf.put(weighted_img)
        out_buf.put(image)
        print("Image in Output Buffer - PID: " + str(os.getpid()))
        imgs += 1
    print("Completed process image for"+ str(imgs)+ "images")
    #return weighted_img
    #return myline

# for source_img in os.listdir("test_images/"):
#     image = mpimg.imread("test_images/"+source_img)
#     processed = process_frame(image)
#     mpimg.imsave("test_images/annotated_"+source_img,processed)

# Code to find frames per second
# start = time.time()
# count = 0
# while(time.time() - start < 1):
#     image = mpimg.imread("road_sample_4.jpg")
#     processed = processImage(image)
#     count += 1
# print("Num Images per second = " + str(count));
# processed = processImage(image)
# mpimg.imsave("road_sample_processed_4.jpg", processed)

# Code to find timing measurements
# start = time.time()
# image = mpimg.imread("road_sample_4.jpg")
# processed = processImage(image)
# print("Total Time: " + str(time.time() - start) + " sec")
# mpimg.imsave("road_sample_processed_4.jpg", processed)

# Code to test Multiprocessing
if __name__ ==  '__main__':
    start = time.time()

    img_buf = multiprocessing.Queue()
    out_buf = multiprocessing.Queue()

    img_opening_process = Process(target=get_images, args=([img_buf]))
    img_processing_process = Process(target=processImage, args=(img_buf, out_buf))
    img_writing_process = Process(target=write_images, args=[out_buf])


    img_opening_process.start()
    img_processing_process.start()
    img_writing_process.start()

    img_opening_process.join()
    img_processing_process.join()
    img_writing_process.join()



    end = time.time()
    print("Total Time: " + str(end - start) + " sec")
# code to capture video
# import cv2
# vidcap = cv2.VideoCapture('challenge.mp4')
# #success,image = vidcap.read()
# success = True
# count = 0
# while success:
#   #cv2.imwrite("frame%d.jpg" % count, image)     # save frame as JPEG file
#   success,image = vidcap.read()
#   if success:
#     processed = processImage(image)
#     mpimg.imsave("test_images/annotated_"+str(count), processed)
#     print('Read a new frame: ', success)
#     count += 1
# print(count)

### code to find frames/sec
"""
count = 0
t = 0
start = time.time()
while (t < 1):
    t = time.time() - start
    image = mpimg.imread("road_sample_4.jpg")
    processed = process_frame(image)
    mpimg.imsave("road_sample_processed_4.jpg", processed)
    count += 1
print(count)
"""
