#!/usr/bin/env python
import rospy
import numpy
import gi
import thread
import threading
from miro_msgs.msg import platform_mics, core_control, platform_sensors, core_state, core_config, platform_control
from Robot import Robot
from Command import Command
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk,GObject,Gdk,GLib,GdkPixbuf

import cv2

GLib.threads_init()
Gdk.threads_init()
def fmt(x, f):
    s = ""
    x = bytearray(x)
    for i in range(0, len(x)):
        if not i == 0:
            s = s + ", "
        s = s + f.format(x[i])
    return s



class MyWindow:

    def __init__(self):
    	
		self.builder = Gtk.Builder()
		self.builder.add_from_file("MiRoGlade.glade")
		self.win = self.builder.get_object("window1")
		self.win.connect("destroy", self.saveAndQuit)
		self.win.show_all()
		
		self.rob = Robot()
		"""self.rob.last_command=Command("right")
		self.rob.last_command.shape = "circle"
		self.rob.last_command.colour = "red"
		self.rob.known_commands = [Command("left"),Command("left"),self.rob.last_command]
		self.rob.state = "Waiting for command" """
		
		self.emotionLab = self.builder.get_object("label1")
		self.moodLab = self.builder.get_object("label2")
		self.stateLab = self.builder.get_object("label3")
		self.leftCommandBox = self.builder.get_object("vbox2")
		self.rightCommandBox = self.builder.get_object("vbox3")
		self.buttonBox = self.builder.get_object("buttons")
		self.curCommandActions = self.builder.get_object("commandInfo")
		self.curCommand = ""
		
		self.cameraL = self.builder.get_object("cameraL")
		#self.cameraL.set_from_file("triangle.png")
		self.cameraR = self.builder.get_object("cameraR")
		#topic_root = "/miro/rob01" #+ self.opt.robot_name
		#self.sub_sensors = rospy.Subscriber(topic_root + "/platform/sensors",platform_sensors, self.callback_platform_sensors)
		
		
		self.commandList = []
		GLib.timeout_add(100, self.update)
		


    def on_button_clicked(self, widget):
    	
    	self.curCommand = widget.get_label()
    
	# save only used for manual testing
    def saveAndQuit(self, gtkobject, data=None):
		fn = open("save.txt", "w")
		
		for d in self.rob.known_commands:
			
			actions = ""
			
			for a in d.sequence:
				actions += ("\n" + str(a.toPrint))
				
			fn.write(str(d.toPrint()) + " " + str(actions) + "\n")
		Gtk.main_quit()
		
    	
    def loop(self):
    	while True:
    		self.rob.emotion+=1
    		print (self.rob.emotion)
    		
    def cvToPixbuf(self, img):
    	h, w, d = img.shape
    	pb = GdkPixbuf.Pixbuf.new_from_data(img.tostring(), GdkPixbuf.Colorspace.RGB,  False, 8, w, h, w*3)
    	return pb
        
    def update(self):

		#self.cameraL.
    	text = "Emotion: " + str(self.rob.emotion) + " / 1"
    	self.emotionLab.set_text(text)
    	text = "Mood: " + str(self.rob.mood) + " / 1"
    	self.moodLab.set_text(text)
    	if len(self.rob.known_commands) > len(self.commandList):
    		self.commandList.append(self.rob.known_commands[-1])
    		text = str(self.commandList[-1].camera) + str(self.commandList[-1].colour) + str(self.commandList[-1].shape)
    		img = Gtk.Image()
    		img.set_from_file(text+".jpg")
    		img2 = Gtk.Image()
    		img2.set_from_file("nothing.png")
    		if self.commandList[-1].camera == "left":
    			self.leftCommandBox.pack_end(img, True, True, 10)
    			self.rightCommandBox.pack_end(img2, True, True, 10 )    			
    			
    		elif self.commandList[-1].camera == "right":
    			self.leftCommandBox.pack_end(img2, True, True, 10)
    			self.rightCommandBox.pack_end(img, True, True, 10 )
    		
    		button = Gtk.Button(text)
    		button.connect("clicked", self.on_button_clicked)
    		self.buttonBox.pack_end(button, True, True, 10)
    		
    	if not self.curCommand == "":
    		text = "Chosen command: " + str(self.curCommand) + "\n\n"
    		for c in self.rob.known_commands:
    			cText = c.camera + c.colour + c.shape
    			if cText == self.curCommand:
    				text += c.actionsToPrint()
    		self.curCommandActions.set_text(text)
    		
    	#text = self.rob.last_command.actionsToPrint()
    	#self.curCommandActions.set_text(text)
    	text = "State: " + str(self.rob.state)
    	self.stateLab.set_text(text)
    	
    	self.cameraL.set_from_file("curleft.jpg")
    	self.cameraR.set_from_file("curright.jpg")
    	#imgg = cv2.imread("triangle.png")
    	#self.cameraL.set_from_pixbuf(self.cvToPixbuf(imgg))
    	
    	
    	self.win.show_all()
    	return True # IMPORTANT
        
    
        

if __name__ == "__main__":

	rospy.init_node("miro_ros_client_py", anonymous=True)
	main = MyWindow()
	#thread.start_new_thread(main.loop, ())
	Gtk.main()
	

	
	

