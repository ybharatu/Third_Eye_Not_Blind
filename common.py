#################################################################
# Global Constants that can be accessed across modules
#################################################################

#################################################################
# Multiprocessing Information
#################################################################
NUM_WORKERS = 2
NUM_FRAMES = 120

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

#################################################################
# Other Global Variables
#################################################################
out_ext = ".jpg"
num_drifts_thresh = 1
