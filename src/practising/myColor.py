#!/usr/bin/env python

import numpy as np
import cv2
import argparse


if __name__ == "__main__":
	
	ap = argparse.ArgumentParser()
	ap.add_argument("-i", "--image", required=True, help="path to the image")
	args = vars(ap.parse_args())
	
	image = cv2.imread(args["image"])
	
	boundaries = [
		([0, 0, 100], [80, 80, 255]),			#red  b,g,r
		([0, 150, 0], [80,255, 80]),			#green
		([100, 0, 0], [255, 70, 70]),		#blue
		#([103, 86, 65], [145, 133, 128])
	]
	num = 0
	result = 0 ## ADDED
	for (lower, upper) in boundaries:
		num+=1
		# create NumPy arrays from the boundaries
		lower = np.array(lower, dtype = "uint8")
		upper = np.array(upper, dtype = "uint8")
	 
		# find the colors within the specified boundaries and apply
		# the mask
		mask = cv2.inRange(image, lower, upper)
		output = cv2.bitwise_and(image, image, mask = mask)
		result = cv2.add(result,output) ## ADDED
		name = "img" + str(num) +".jpg"
		# show the images
		#cv2.imshow("images", np.hstack([image, output]))
		cv2.imshow("images", result)
		cv2.imwrite(name, output)
		cv2.waitKey(0)

