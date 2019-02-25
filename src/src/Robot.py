#!/usr/bin/env python

"""
This is a Robot class of the MiRo project
"""

import rospy
import time

from std_msgs.msg import String
from geometry_msgs.msg import Twist

import miro_msgs
from miro_msgs.msg import platform_mics, core_control, platform_sensors, core_state, core_config, platform_control
from miro_constants import miro

from os import system, name 

def fmt(x, f):
    s = ""
    x = bytearray(x)
    for i in range(0, len(x)):
        if not i == 0:
            s = s + ", "
        s = s + f.format(x[i])
    return s

class Robot:
	
	def callback_platform_sensors(self, object):
		
		if not self.active:
			return
			
		self.platform_sensors = object
		
		q = platform_control()
		
		self.updateEmotion(self.detectTouch())
		self.updateMood()
		
		q.tail = round(self.emotion,1)
		self.pub_platform_control.publish(q)
		
		self.count += 1
	
	def callback_platform_mics(self, object):
		
		if not self.active:
			return
		
		self.mics = object
		if (self.count/50)<3: # for 3 sec, note: this only works if command time smaller than mood update time!
			self.storedSound[self.count/50] = self.mics.data
		else:
			fn = open("goForward.txt", "w")
			for d in self.storedSound:
				for i in range(4000):
					fn.write(str(d[i])+"\n")
		
	def detectCommand(self):
		return
	
	def addCommand(self, c):
		self.known_commands.append(c)
	
	def updateEmotion(self, isTouched):
		
		if ( isTouched and self.emotion < 1 ):
			self.emotion += 0.0005
		elif (not isTouched):
			self.emotion -= 0.0001
		#print("emotion: " + str(self.emotion))
		
	def updateMood(self):
		
		sync_rate = 50
		time = self.count / sync_rate

		if ( time > 5 ) and self.isHappy():
			self.mood+=0.1
			self.count = 0
			print "mood: " + str(self.mood) + "time: " + str(time)
		elif (time>5) and (not self.isHappy()):
			self.mood-=0.1
			print "mood: " + str(self.mood) + "time: " + str(self.count)
			self.count = 0
			
	def isHappy(self):
		return self.emotion > 0
		
		
	def detectTouch(self):
		
		for sensor in self.platform_sensors.touch_body:
			if fmt(sensor, '{0:.0f}') == '1':
				return True
		for sensor in self.platform_sensors.touch_head:
			if fmt(sensor, '{0:.0f}') == '1':
				return True
		return False
	

	def __init__(self):
		
		self.platform_sensors = None
		self.mics = None
		self.known_commands = []
		self.last_emotion = 0.0
		self.emotion = 0.0
		self.mood = 0.0
		self.count = 0
		#self.size = 4000*3
		self.storedSound =[None] * 3
		#self.curSoundIndex = 0
		
		#publish
		topic_root = "/miro/rob01"
		self.pub_platform_control = rospy.Publisher(topic_root + "/platform/control", platform_control, queue_size=0)
        
		#subscribe
		self.sub_sensors = rospy.Subscriber(topic_root + "/platform/sensors", platform_sensors, self.callback_platform_sensors)
		self.sub_mics = rospy.Subscriber(topic_root + "/platform/mics", platform_mics, self.callback_platform_mics)
        
		self.active = True
