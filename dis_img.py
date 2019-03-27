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
# import picamera
# from picamera.array import PiRGBArray
import io
import subprocess

ih = subprocess.Popen(["feh", "source_images/live_0.jpg"])
time.sleep(1)
ih.kill()


