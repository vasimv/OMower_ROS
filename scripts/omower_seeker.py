#!/usr/bin/env python
""" Chessboard detecting script (omower-seeker)
"""
__author__ =  'Vasim V. <vasimv@gmail.com'
__version__=  '0.1'
__license__ = 'GLPv3'
# Python libs
import sys, time
from sys import argv
import numpy as np
import cv2
import glob
import time
import sys
import math
from std_msgs.msg import UInt8MultiArray

import roslib
import rospy

# Camera topic name
cameraTopic = "/camera/image/compressed"
# pfodApp command output topic (OMower's commands input)
outputTopic = "/cmdIn"
# Print debug info
debug = True
# last time when chessboard was recognized (in millseconds)
lastVisible = 0
# distance to the chessboard, in centimeters
distance = 0
# Offset angle of chessboard (degrees, -90..90)
offsetAngle = 0
# Orientation angle of chessboard (-90..90)
orientAngle = 0
# Time of measurement (age of chessboard's fix)
age = 0
# Number of frames processed
nFrames = 0
# Time to recognize chessboard
timeRecognize = 0
from sensor_msgs.msg import CompressedImage

def millis():
  return time.time() * 1000

def callback(ros_data):
  global lastVisible
  global distance
  global offsetAngle
  global orientAngle
  global nFrames
  global age
  global debug
  global cameraResolution
  global cameraRotation
  global pubCmd

  timeMark = millis()
  np_arr = np.fromstring(ros_data.data, np.uint8)
  img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
  picHeight, picWidth = img.shape[:2]
  # Split stereo camera image if needed (use left one)
  if (picWidth / picHeight) >= 3:
    img = img[0:picHeight, 0:(picWidth / 2)]
    picWidth = picWidth / 2
  # Find the chess board corners with OpenCV
  ret, corners = cv2.findChessboardCorners(img, (4,3), flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
  if debug:
    print "findchessboard ms: %d" % (millis() - timeMark)
  if ret == True:
    # Found chessboard, let's calculate stuff
    x1 = corners[0][0][0]
    x2 = corners[8][0][0]
    x3 = corners[3][0][0]
    x4 = corners[11][0][0]
    y1 = corners[0][0][1]
    y2 = corners[3][0][1]
    y3 = corners[8][0][1]
    y4 = corners[11][0][1]
    center = (x1 + x2) / 2
    sizeX = abs(x2 - x1)
    sizeX2 = abs(x4 - x3);
    sizeY = abs(y2 - y1)
    sizeY2 = abs(y4 - y3)
    avgHeight = (sizeY + sizeY2) / 2
    avgWidth = (sizeX + sizeX2) / 2
    ratioXY = avgHeight / avgWidth

    angle1 = (picWidth - center) / 7.5
    # Calculate chessboard orientation
    if ratioXY < 1.5:
      angle2 = 0
    else:
      # Calculate triangle angle (side B is avgWidth)
      sideC = avgHeight / 1.5
      if (sideC < avgWidth):
        sideA = math.sqrt(avgWidth ** 2 - sideC ** 2) 
      else:
        sideA = math.sqrt(sideC ** 2 - avgWidth ** 2) 
      angle2 = (180 * (math.pi - math.acos((sideA ** 2 - avgWidth ** 2 - sideC ** 2) / (2 * avgWidth * sideC)))) / math.pi
    if (sizeY < sizeY2):
        angle2 = -angle2

    # Distance calculation for RPI camera v2 module and chessboard with square size 47x47mm
    if (picWidth == 1280):
      dist = 10680 / avgHeight
      angle1 = (picWidth / 2 - center) / 10.68
    elif (picWidth == 640) or (picWidth == 672):
      dist = 5340 / avgHeight
      angle1 = (picWidth / 2 - center) / 5.34
    elif picWidth == 480:
      dist = 3560 / avgHeight
      angle1 = (picWidth / 2 - center) / 3.56
    else:
      dist = 2670 / avgHeight
      angle1 = (picWidth / 2 - center) / 2.67

    if debug:
      print "sizeX, sizeY %f %f" % (sizeX, sizeY)
      print "avgHeight %f" % avgHeight
      print "center %f" % center
      print "ratioXY: %f" % ratioXY
      print "angle1: %f" % angle1
      print "angle2: %f" % angle2
      print "distance %d" % dist

    # Put result values into global variables for IO thread
    timeRecognize = millis() - timeMark
    if (timeRecognize < 500):
      distance = dist
      offsetAngle = angle1
      orientAngle = angle2
      lastVisible = millis()
      age = timeRecognize + 100
      lastWrote = lastVisible
      msgOut = "{g01`%d,%d,%d,%d}" % (offsetAngle, orientAngle, distance, age)
      #pubCmd.publish(UInt8MultiArray(data = bytearray(msgOut))
      arrOut = UInt8MultiArray()
      arrOut.data = list(bytearray(msgOut))
      pubCmd.publish(arrOut)
  nFrames = nFrames + 1
  if debug:
    print "total ms: %d" % (millis() - timeMark)

# Main code

rospy.init_node('omower_seeker', anonymous=True)
subscriber = rospy.Subscriber(cameraTopic, CompressedImage, callback, queue_size = 1, buff_size=20480000)
pubCmd = rospy.Publisher(outputTopic, UInt8MultiArray, queue_size = 1)
rospy.Rate(0.05)

lastWrote = lastVisible
msgOut = ""

print "starting server"
while True:
  rospy.spin()
