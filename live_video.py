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


cap = cv2.VideoCapture(0)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    """
    # Display the resulting frame
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    """
    print("image processed")
    time.sleep(0.5)

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
