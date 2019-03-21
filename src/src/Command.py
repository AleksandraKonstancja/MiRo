#!/usr/bin/env python

"""
This is a Command class of the MiRo project
"""
import cv2
import numpy as np
import argparse
from ActionManager import ActionManager

boundaries = {
			"red" : ([0, 0, 90], [60, 60, 255]),			
			"green" : ([0, 90, 0], [80,255, 80]),			
			"blue" : ([81, 0, 0], [255, 89, 89]),		
		}

class Command:
	
		
	def startSequence(self):
		return
		
	def addAction(self, a):
		return
		
	"""
	Updates probability of last action performed for the command based on
	the learning rate.
	"""
	def updateProbs(self, learnRate):
		print self.action.last_action
		if not self.action.last_action == "":
			self.action.updateProbs(learnRate)
	
	"""
	Gets shape and colour of the largest shape in an image
	"""
	def getCommandData(self,image):
		img = self.prepareImage(image)
		cnts = self.detectShape(img)
		
		if len(cnts)>0:
			shape = self.recognizeShape(cnts)
			cv2.drawContours(img, [cnts],-1,(0,255,0),2)
			self.shape = shape
			#print self.shape
			self.detectColour(img, cnts)
		#else:
			#print " no shape detected"
	
	
	"""
	Prepares an image for detection by adjusting brightness and saturation, 
	and applying mask that only leaves red, green or blue parts
	"""
	def prepareImage(self, image):
		
		cv2.imshow(self.camera, image)
		cv2.waitKey(1) & 0xFF
		
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		(h,s,v) = cv2.split(hsv)
		s = s+30
		v = v+20
		hsv = cv2.merge([h,s,v])
		image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
		final = 0
	
		num = 0
		for (lower, upper) in boundaries.values():
			num+=1
			# create NumPy arrays from the boundaries
			lower = np.array(lower, dtype = "uint8")
			upper = np.array(upper, dtype = "uint8")
		 
			# find the colors within the specified boundaries and apply the mask
			mask = cv2.inRange(image, lower, upper)
			output = cv2.bitwise_and(image, image, mask = mask)
			name = "img" + str(num) +".jpg"
			final = cv2.add(final,output)
			
			
		return final
			
	"""
	Decides a shape based on detected contours
	"""
	def recognizeShape(self, c):
		shape = ""
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.04 * peri, True)
		
		if len(approx) ==3:
			shape = "triangle"
		elif len(approx) ==4:
			shape = "square"
		else:
			shape = "circle"
		
		return shape
		
	"""
	Finds contours of the biggest shape on the image
	"""
	def detectShape(self, image):
			
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		blurred = cv2.GaussianBlur(gray,(31,31),0)
		thresh = cv2.threshold(blurred, 20, 255, cv2.THRESH_BINARY)[1]
		
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
			
		if biggest_area > 1000: 
			return biggest_shape
		
		return []
	
	"""
	Check colour in the centre of given contours	
	"""
	def detectColour(self, image, cnts):
		
			m = cv2.moments(cnts)
			x = int(m["m10"] / m["m00"])
			y = int(m["m01"] / m["m00"])
			
			centreColour = image[y,x]
			number = 0
			for (lower, upper) in boundaries.values():
				
				lower = np.array(lower, dtype = "uint8")
				upper = np.array(upper, dtype = "uint8")
					
				correct = 0
				for i in range(3):
					if centreColour[i]>lower[i] and centreColour[i]<upper[i]:
						correct +=1
				
				if correct==3:
					#print "color: " + str(boundaries.keys()[number])
					self.colour = str(boundaries.keys()[number])
					
				number+=1
			
			text = str( self.colour) + " " + str(self.shape) 
			cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)	
			text2 = str(self.camera) + " shape detected" 
			cv2.imshow(text2, image)
			cv2.waitKey(1) & 0xFF
			#cv2.destroyAllWindows()
			
	
	def performAction(self):
		return self.action.performAction()
	
	"""
	Checks that the object has all necessary information to be a command
	"""
	def isCommand(self):
		
		if self.shape == "" or self.colour == "" or self.camera == "":
			return False
		else:
			return True
	
	"""
	Checks that two command objects are the same based on shape, colour
	and camera only 
	"""
	def equals(self, other):
		
		if self.camera == other.camera and self.shape == other.shape and self.colour == other.colour:
			return True
		else:
			return False
			
	def toPrint(self):
		return (str(self.camera) + " " + str(self.colour) + " " + str(self.shape))
	
	def __init__(self, camera):
		
		#self.command_data = None
		self.camera = camera
		self.shape = ""
		self.colour = ""
		self.action = ActionManager()
		self.sequence = []
		
"""if __name__ == "__main__":
	#comm =Command()
	#images = comm.detectColour("redTriangle.png")
	#comm.detectShape(images)"""