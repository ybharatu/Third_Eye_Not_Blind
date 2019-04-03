import matplotlib
matplotlib.use('PS')
import matplotlib.image as mpimg
import picamera
from picamera.array import PiRGBArray
import numpy as np
import time
import io
from PIL import Image
import cv2

NUM_FRAMES = 5

in_imgs = 0
with picamera.PiCamera() as camera:
    camera.resolution = (640,480)
    rawCapture = PiRGBArray(camera)
    time.sleep(0.1) #wait for camera to warm up
    start = time.time()
    for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
        filepath = "videos/image" + str(in_imgs) + str(".jpg")
        img = frame.array
        rawCapture.truncate(0)
        mpimg.imsave(filepath, img)
        print("img " + str(in_imgs))
        in_imgs += 1
        if in_imgs > NUM_FRAMES:
            break
    total_time = time.time() - start
    print("frames per sec: " + str(in_imgs/total_time))


