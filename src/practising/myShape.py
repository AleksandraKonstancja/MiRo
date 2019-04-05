#!/usr/bin/env python

import numpy as np
import cv2
import argparse
#import imutils

class Shape:
	
	def __init__(self):
		pass
	
	def detect(self, c):
		shape = "not recognized"
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.04 * peri, True)
		
		if len(approx) ==3:
			shape = "triangle"
		else:
			shape = "circle"
		
		return shape
		

if __name__ == "__main__":
	
	ap = argparse.ArgumentParser()
	ap.add_argument("-i", "--image", required=True, help="path to the image")
	args = vars(ap.parse_args())
	
	image = cv2.imread(args["image"])
	resized = image #imutils.resize(image, width=300)
	ratio = image.shape[0] / float(resized.shape[0])
	
	gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
	blurred = cv2.GaussianBlur(gray,(11,11),0)
	thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
	
	cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	#cnts = cv2.findContours(blurred.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	if len(cnts) ==2:
		cnts = cnts[0]
	elif len(cnts) ==3:
		cnts = cnts[1]
	else:
		print "error"

	sd = Shape()
	biggest_area = 0
	biggest_shape = None
	
	for c in cnts:
		
		cur_area = cv2.contourArea(c)
		
		if cur_area > biggest_area:
			biggest_area = cur_area
			biggest_shape = c
		
		"""shape = sd.detect(c)
		c=c.astype("float")
		c *= ratio
		c = c.astype("int")
		cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
		print shape
		print cv2.contourArea(c)"""
	
	shape = sd.detect(biggest_shape)
	cv2.drawContours(image, [biggest_shape],-1,(0,255,0),2)
	print shape
		
		
	
	cv2.imshow("image", image)
	cv2.waitKey(0)
		

