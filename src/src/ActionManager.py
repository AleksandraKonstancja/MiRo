#!/usr/bin/env python

"""
This is a ActionManager class of the MiRo project
"""
import unittest

class ActionManager:
	
	def updateProbs(self):
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
		initial_prob = 100 / self.actionNumber
		
		for key in self.probabilityDict.keys():
			self.probabilityDict[key] = initial_prob
			
		
class TestClass(unittest.TestCase):
	
	def test_init(self):
		am = ActionManager()
		for key in am.probabilityDict.keys():
			self.assertTrue(am.probabilityDict[key] == 100/6)
			
	def test_highestProb(self):
		am = ActionManager()
		am.probabilityDict["go"] = 50
		self.assertTrue(am.findHighestProb() == "go")
		
			
if __name__ == '__main__':
	unittest.main()




