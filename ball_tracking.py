# USAGE
# python ball_tracking.py --video ball_tracking_example.mp4
# python ball_tracking.py

# import the necessary packages
from collections import deque
from picamera.array import PiRGBArray
from picamera import PiCamera
import sys
PY3 = sys.version_info[0] == 3
if PY3:
    xrange = range
import numpy as np
import argparse
#import imutils
import cv2
import serial
import time

# construct serial port
ser = serial.Serial(port='/dev/serial0', baudrate=9600, bytesize=8, parity='N', stopbits=1)

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
#ap.add_argument("-v", "--video",
#	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points

# ROOM
#greenLower = (22, 43, 84)
#greenUpper = (46, 195, 222)

# DOWNSTAIRS
greenLower = (41, 51, 48)
greenUpper = (78, 131, 243)
#greenLower = (32, 48, 42)
#greenUpper = (60, 165, 255)

# LAB
#greenLower = (34, 51, 38)
#greenUpper = (53, 238, 226)

pts = deque(maxlen=args["buffer"])

#PiCamera init
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)
# if a video path was not supplied, grab the reference
# to the webcam
#if not args.get("video", False):
#	camera = cv2.VideoCapture(0)

# otherwise, grab a reference to the video file
#else:
#	camera = cv2.VideoCapture(args["video"])

# keep looping
for frames in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the current frame
	frame = frames.array
	frame = cv2.flip(frame,0)
	frame = cv2.flip(frame,1)

	# if the 'q' key is pressed, stop the loop
	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"):
            cv2.imwrite(r'/home/pi/Pictures/test.png', frame)
            break
        
	#(grabbed, frame) = camera.read()

	# if we are viewing a video and we did not grab a frame,
	# then we have reached the end of the video
	#if args.get("video") and not grabbed:
	#	break

	# resize the frame, blur it, and convert it to the HSV
	# color space
	#frame = cv2.resize(frame, None, fx=0.95, fy=0.95, interpolation = cv2.INTER_AREA)
	# blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
	center = None

	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		# only proceed if the radius meets a minimum size
		if radius > 25:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)
			# split up 'x' data and send over serial
			#print(x)
			hpos = int(x)
			hpos1 = hpos & 0xF
			ser.write('A'.encode())
			ser.write(bytes([hpos1]))
			hpos2 = (hpos & 0xF0) >> 4
			ser.write('B'.encode())
			ser.write(bytes([hpos2]))
			hpos3 = (hpos & 0x300) >> 8
			ser.write('C'.encode())
			ser.write(bytes([hpos3]))
			#print(str(hpos1) + ' ' + str(hpos2) + ' ' + str(hpos3))
			# split up radius data and send over serial
			rad = int(radius)
			rad1 = rad & 0xF
			ser.write('X'.encode())
			ser.write(bytes([rad1]))
			rad2 = (rad & 0xF0) >> 4
			ser.write('Y'.encode())
			ser.write(bytes([rad2]))
			#print(rad)
			#print(str(rad1) + ' ' + str(rad2))

	# update the points queue
	pts.appendleft(center)

	# loop over the set of tracked points
	for i in xrange(1, len(pts)):
		# if either of the tracked points are None, ignore
		# them
		if pts[i - 1] is None or pts[i] is None:
			continue

		# otherwise, compute the thickness of the line and
		# draw the connecting lines
		thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
		cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

	# show the frame to our screen
	cv2.imshow("Frame", frame)

	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)

# cleanup the camera and close any open windows
#camera.release()
cv2.destroyAllWindows()
