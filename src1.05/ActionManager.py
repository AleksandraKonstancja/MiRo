#!/usr/bin/env python

"""
This is a ActionManager class of the MiRo project
"""
import unittest
import random
from time import time
from miro_msgs.msg import platform_control
from miro_constants import miro

class ActionManager:
	
	"""
	Updates last action by the learning rate and recalculates remaining 
	probabilities depending on previous contribution to the total ( 100 )
	of each action.
	learn_ratio - how much should action be affected
	"""
	def updateProbs(self, learn_ratio):
		
		action = self.choice
		prob_sum = learn_ratio + self.probabilityDict[action]
		
		# if probability of an action is 100 it cannot get higher - no need to update
		# this allows avoiding division by 0 when prev_rest is 0
		if self.probabilityDict[action] == 100.0 and learn_ratio > 0:
			self.isFullyLearned = True
			return
		
		# make sure max probability is 100
		if prob_sum > 100.0:
			learn_ratio = 100 - self.probabilityDict[action]
		
		prev_rest = 100 - self.probabilityDict[action]
		
		# change probabilities proportionally to previous values
		self.probabilityDict[action] += learn_ratio
		rest = 100 - self.probabilityDict[action]
		
		for key in self.probabilityDict.keys():
			if not key == action:
				rest_rate = rest / prev_rest
				self.probabilityDict[key] *= rest_rate
			
		#print self.probabilityDict
	
	"""
	Performs an action based on its probability
	"""
	def performAction(self):
		if self.isStopAction:
			self.stop()
		else:
			total_prob = 0
			randomNum = random.randint(1,101)
			
			for key in self.probabilityDict.keys():
				total_prob += self.probabilityDict[key]
				if randomNum < total_prob:
					self.actionDict[key]()
					break
		return self.q
				
	"""
	Performs most likely action
	"""
	def performMostLikelyAction(self):
		self.actionDict[self.findHighestProb()]
		return
		
	def isFullyLearned(self):
		prob = self.probabilityDict[self.choice]
		return (prob == 100)
		
	def isStopAction(self):
		return self.isStopAction
		
	def toPrint(self):
		if self.isStopAction:
			info = "Stop"
		else:
			action = self.findHighestProb()
			info = "Most likely action: " + str(action) + " ( " + str(round(self.probabilityDict[action],1)) + " )\t Choice: " + str(self.choice) +"\n"
			for key in self.probabilityDict:
				info+= key + ": " + str(round(self.probabilityDict[key],1)) + ", "
		return info
		
	
	"""
	Finds an action with highest probability of being performed
	"""	
	def findHighestProb(self):
		return max(self.probabilityDict, key=self.probabilityDict.get)
	
	"""
	Set of functions responsible for interacting with robot's actuators
	based on action chosen to be performed.
	"""
	def turnLeft(self):
		self.q.body_vel.angular.z = +0.5235 #0.7854
		self.choice = "turnL"
		print "left"
	def turnRight(self):
		self.q.body_vel.angular.z = -0.5235 #0.7854
		self.choice = "turnR"
		print "right"
	def go(self):
		self.q.body_vel.linear.x = 100
		self.choice = "go"
		print "go"
	def eyes(self):
		self.q.eyelid_closure = 1
		self.choice = "eyes"
		print "eyes"
	def headDown(self):
		self.q.body_config[1] =  miro.MIRO_LIFT_MAX_RAD - self.q.body_config[1]
		self.q.body_config_speed[1] = miro.MIRO_P2U_W_LEAN_SPEED_INF
		self.choice = "headDown"
		print "hDown"
	def ears(self):
		self.q.ear_rotate[0] = 1 
		self.q.ear_rotate[1] = 1 
		self.choice = "ears"
		print "ears"
	def stop(self):
		self.q = platform_control()
		self.q.body_config[1] =  miro.MIRO_LIFT_MIN_RAD - self.q.body_config[1]
		self.q.body_config_speed[1] = miro.MIRO_P2U_W_LEAN_SPEED_INF
		self.q.eyelid_closure = 0
		self.q.body_vel.linear.x = 0
		self.q.body_vel.angular.z = 0
		self.choice = "stop"
		print "stop"
			
	def __init__(self, isStop=False):
		
		self.choice = ""
		self.isFullyLearned = False
		
		self.q = platform_control()
		
		self.actionDict = {
			"turnL" : self.turnLeft,
			"turnR" : self.turnRight,
			"go" : self.go,
			"eyes" : self.eyes,
			"headDown" : self.headDown,
			"ears" : self.ears,
		}
		
		self.probabilityDict = {}
		
		self.isStopAction = isStop
		
		self.actionNumber = len(self.actionDict)
		initial_prob = 100.0 / self.actionNumber
			
		for key in self.actionDict.keys():
			self.probabilityDict[key] = initial_prob
			
			
			
		
