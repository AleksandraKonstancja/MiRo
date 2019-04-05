import cv2
import video
import common
import sys
import numpy as np
from Command import Command
import time

cv2.namedWindow('cameraview')

cam = video.create_capture(0)


while True:
   # read an image from the video camera
   ret, img = cam.read()
   com = Command("test")
   com.getCommandData(img)
   com.performAction()
   cur = time.time()
   end = cur+2
   print "started waiting for feedback"
   while (cur < end):
   	   cur = time.time()
   print "stopped waiting for feedback"
  
   # put that image in the window 
   cv2.imshow('cameraview', img)

   # listen for a key (waitKey also redraws image, so is pretty
   # important. if the key is 27, quit 
   if 0xFF & cv2.waitKey(5) == 27:
      break



