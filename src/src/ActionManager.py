#!/usr/bin/env python

"""
This is a ActionManager class of the MiRo project
"""
import unittest

class ActionManager:
	
	def updateProbs(self, action, learn_rate):
		for key in self.probabilityDict.keys():
			if key == action:
				self.probabilityDict[key] += learn_rate
			else:
				self.probabilityDict[key] -= (learn_rate / (self.actionNumber-1))
		return
		
	def performAction(self):
		self.actionDict[findHighestProb()]
		return
		
	def findHighestProb(self):
		return max(self.probabilityDict, key=self.probabilityDict.get)
	
	def turnLeft(self):
		return
	def turnRight(self):
		return
	def go(self):
		return
	def eyes(self):
		return
	def headDown(self):
		return
	def ears(self):
		return
			
	def __init__(self):
		
		self.actionDict = {
			"turnL" : self.turnLeft,
			"turnR" : self.turnRight,
			"go" : self.go,
			"eyes" : self.eyes,
			"headDown" : self.headDown,
			"ears" : self.ears,
		}
		
		self.probabilityDict = {
			"turnL" : None,
			"turnR" : None,
			"go" : None,
			"eyes" : None,
			"headDown" : None,
			"ears" : None,
		}
		
		
		self.actionNumber = len(self.probabilityDict)
		initial_prob = 100.0 / self.actionNumber
		
		for key in self.probabilityDict.keys():
			self.probabilityDict[key] = initial_prob
			
		
class TestClass(unittest.TestCase):
	
	def test_init(self):
		am = ActionManager()
		for key in am.probabilityDict.keys():
			print "initial prob: " + str(am.probabilityDict[key])
			self.assertTrue(am.probabilityDict[key] == 100.0/6)
			
	def test_highestProb(self):
		am = ActionManager()
		am.probabilityDict["go"] = 50
		self.assertTrue(am.findHighestProb() == "go")
		
	def test_updateProbs(self):
		am = ActionManager()
		am.updateProbs("go", 10)
		prob_sum = 0
		for key in am.probabilityDict.keys():
			prob_sum += am.probabilityDict[key]
		
		print "sum: " + str(prob_sum)
		self.assertTrue(prob_sum == 100.0)
		
			
if __name__ == '__main__':
	unittest.main()




