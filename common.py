#################################################################
# Import Packages
#################################################################
import multiprocessing


#################################################################
# Global Constants that can be accessed across modules
#################################################################

#################################################################
# Multiprocessing Information
#################################################################
NUM_WORKERS = 2
NUM_FRAMES = 200

#################################################################
# GPIO Pin Values
#################################################################
DRIFT_LEFT_PIN = 11
DRIFT_RIGHT_PIN = 13
ERROR_PIN = 15
chan_list = [DRIFT_LEFT_PIN, DRIFT_RIGHT_PIN, ERROR_PIN]

#################################################################
# Lists used for draw_lines
#################################################################
rightSlope, leftSlope, rightIntercept, leftIntercept = [], [], [], []
rightSlopeDL, leftSlopeDL, rightInterceptDL, leftInterceptDL = [], [], [], []
# rightSlopeDL = multiprocessing.Queue(5)
# leftSlopeDL = multiprocessing.Queue(5)
# rightInterceptDL = multiprocessing.Queue(5)
# leftInterceptDL = multiprocessing.Queue(5)
num_frame_persist = 10
rightTTL = num_frame_persist
leftTTL = num_frame_persist

#################################################################
# Other Global Variables
#################################################################
out_ext = ".jpg"
num_drifts_thresh = 3
gpio_flag = True
