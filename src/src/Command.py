#!/usr/bin/env python

"""
This is a Command class of the MiRo project
"""
import cv2
import numpy as np
import argparse

class Command:
	
	def compare(self, other):
		return
		
	def startSequence(self):
		return
		
	def addAction(self, a):
		return
		
	def detectColour(self, image):
		
		cv2.imshow("image", image)
		cv2.waitKey(0)
		#image = bridge.imgmsg_to_cv2(imgMsg) #cv2.imread(img)
		images = []
	
		boundaries = [
			([0, 0, 100], [80, 80, 255]),			#red  b,g,r
			([0, 150, 0], [80,255, 80]),			#green
			([100, 0, 0], [255, 70, 70]),		#blue
		]
		num = 0
		for (lower, upper) in boundaries:
			num+=1
			# create NumPy arrays from the boundaries
			lower = np.array(lower, dtype = "uint8")
			upper = np.array(upper, dtype = "uint8")
		 
			# find the colors within the specified boundaries and apply
			# the mask
			mask = cv2.inRange(image, lower, upper)
			output = cv2.bitwise_and(image, image, mask = mask)
			name = "img" + str(num) +".jpg"
			images.append(output)
			
		return images
			
		
	def recognizeShape(self, c):
		shape = "not recognized"
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.04 * peri, True)
		
		if len(approx) ==3:
			shape = "triangle"
		else:
			shape = "circle"
		
		return shape
	
	def detectShape(self, imgs):
		
		for image in imgs:
			
			gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
			blurred = cv2.GaussianBlur(gray,(21,21),0)
			thresh = cv2.threshold(blurred, 40, 255, cv2.THRESH_BINARY)[1]
			
			cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		
			if len(cnts) ==2:
				cnts = cnts[0]
			elif len(cnts) ==3:
				cnts = cnts[1]
			else:
				print "error"
		
			biggest_area = 0
			biggest_shape = None
			
			for c in cnts:
				
				cur_area = cv2.contourArea(c)
				
				if cur_area > biggest_area:
					biggest_area = cur_area
					biggest_shape = c
				
			if biggest_area > 500: # why this number
				shape = self.recognizeShape(biggest_shape)
				cv2.drawContours(image, [biggest_shape],-1,(0,255,0),2)
				print shape
			else:
				print " no shape detected"
				
				
			
			#cv2.imshow("image", image)
			#cv2.waitKey(0)
			
	
	def __init__(self):
		
		#self.command_data = None
		self.shape = ""
		self.color = ""
		self.sequence = []
		
"""if __name__ == "__main__":
	#comm =Command()
	#images = comm.detectColour("redTriangle.png")
	#comm.detectShape(images)"""