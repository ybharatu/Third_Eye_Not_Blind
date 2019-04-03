import picamera
import getopt
import sys


try:
    opts, args = getopt.getopt(sys.argv[1:], "n:f:t:", ["vidnum=", "filename=", "time="])
except getopt.GetoptError:
    print('video_to_file.py -f filename')
    print('OR video_to_file.py -n #')
    sys.exit(2)

#################################################################
# Parses options
#################################################################
vid = False
t = 60
filename = ""
for opt, arg in opts:
    if opt in ("-f", "--filename"):
        filename = arg.strip()
    elif opt in ("-n", "--numvid"):
        vid = True
        num = arg.strip()
    elif opt in ("-t", "--time"):
        t = arg.strip()

with picamera.PiCamera() as camera:
    camera.resolution = (640, 480)
    if vid:
        camera.start_recording("videos/video_" + str(num) + ".h264")
    elif len(filename) != 0:
        camera.start_recording("videos/" + str(filename) + ".h264")
    else:
        print("warning: give the video a name!")
        camera.start_recording("videos/nonamevid.h264")

    camera.wait_recording((int)(t))
    camera.stop_recording()
