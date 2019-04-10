# Adapted from https://github.com/galenballew/SDC-Lane-and-Vehicle-Detection-Tracking

#################################################################
# Import Packages
#################################################################
import os
import sys
from common import *
import time
from multiprocessing import Process
import multiprocessing
import getopt
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO!  This is probably because you need superuser privileges.  You can achieve this by using 'sudo' to run your script")
except ImportError:
    gpio_flag = False
    print("RPi is not installed. It can only be installed on linux enviornments")

#################################################################
# Function: GPIO_setup
# Description: Sets up GPIO pins in chan_list as outputs.
# GPIO.BOARD specifies that you are referring to the pins by the
# number of the pin the the plug - i.e the numbers printed on
# the board. Warnings are enabled
#################################################################
def GPIO_setup():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(True)
    GPIO.setup(chan_list, GPIO.OUT)

#################################################################
# Function: GPIO_cleanup
# Description: Cleans up resources used by GPIO pins in
# chan_list. Returns all channels to be inputs with no pull
# up/down.
# Note: Currently only calls GPIO.cleanup() but here just in case
# something else needs to be done.
#################################################################
def GPIO_cleanup():
    GPIO.cleanup()

#################################################################
# Function: Main Function
# Note: if __name__ == '__main__' guard is necessary in order to
# use processes
#################################################################
def main(argv):
    vid = False
    save = False
    live = False
    im = False

    try:
        opts, args = getopt.getopt(argv, "hv:sli:f:", ["help", "video", "save", "live", "image", "filename="])
    except getopt.GetoptError:
        print('lane_detect.py -i <filename> -v')
        sys.exit(2)

    #################################################################
    # Parses options
    #################################################################
    for opt, arg in opts:
        if opt == '-h':
            print('-v --video: Enables video processing. Usage: python3 lane_detect.py -v <filename>')
            print('-l --live: Enables live video processing. Usage: python3 lane_detect.py -l')
            print('-i --image: Enables still image processing. Usage: python3 lane_detect.py -i <filename>')
            print('-s --save: Saves processed images (Slower Performance).')
            sys.exit()
        elif opt in ("-f", "--filename"):
            filename = arg.strip()
        elif opt in ("-v", "--video"):
            vid = True
            filename = arg.strip()
            from lane_video import handle_images, processImage
            print("Video Processing")
        elif opt in ("-l", "--live"):
            live = True
            filename = "livefeed"
            from lane_live_video import handle_images, processImage
            print("Live Processing")
        elif opt in ("-i", "--image"):
            im = True
            filename = arg.strip()
            from lane_still_image import handle_images, processImage
            print("Image Processing")
        elif opt in ("-s", "--save"):
            print("Saving Image")
            save = True

    #################################################################
    # Exits if main option is not set
    #################################################################
    if not vid and not im and not live:
        print("Please specify input type (Video (-v), Image (-i), or Live (-l)")
        sys.exit()

    print("FILENAME: " + filename)

    #################################################################
    # Sets up GPIO pins
    #################################################################
    if gpio_flag:
        GPIO_setup()

    #################################################################
    # Creates Multiprocessing Queues and processes
    #################################################################
    input_img_buffer_list = []
    output_img_buffer_list = []
    process_list = []
    for i in range(NUM_WORKERS):
        input_img_buffer_list.append(multiprocessing.Queue())
        output_img_buffer_list.append(multiprocessing.Queue())
        process_list.append(Process(target=processImage, args=(input_img_buffer_list[i], output_img_buffer_list[i], save, i)))

    img_handling_process = Process(target=handle_images, args=(input_img_buffer_list,\
                                                            output_img_buffer_list, vid, filename, live, im, save))

    start = time.time()
    #################################################################
    # Starts all the processes
    #################################################################
    img_handling_process.start()
    for i in range(NUM_WORKERS):
        process_list[i].start()

    #################################################################
    # Waits for all the processes to finish
    #################################################################
    img_handling_process.join()
    print("Finished Process 1")
    for i in range(NUM_WORKERS):
        process_list[i].join()
        print("Finished Process " + str(i + 2))

    #################################################################
    # Prints Finished Execution Statistics:
    #################################################################
    end = time.time()
    print("Total Time: " + str(end - start) + " sec")
    print("FPS: " + str(NUM_FRAMES/(end - start)) + " sec")

    #################################################################
    # Cleans up GPIO Resources
    #################################################################
    if gpio_flag:
        GPIO_cleanup()


if __name__ == "__main__":
   main(sys.argv[1:])
