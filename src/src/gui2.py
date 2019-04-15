#!/usr/bin/env python
import rospy
import gi
import thread
import threading
from Robot import Robot
from Command import Command
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk,GObject,Gdk,GLib,GdkPixbuf

GLib.threads_init()
Gdk.threads_init()




class MyWindow:

    def __init__(self):
    	
		self.builder = Gtk.Builder()
		self.builder.add_from_file("ver2.glade")
		self.win = self.builder.get_object("window1")
		self.win.connect("destroy", Gtk.main_quit)
		self.win.show_all()
		
		self.rob = Robot()
		self.rob.last_command=Command("right")
		self.rob.last_command.shape = "circle"
		self.rob.last_command.colour = "red"
		self.rob.known_commands = [Command("left"),Command("left"),self.rob.last_command]
		self.rob.state = "Waiting for command"
		
		self.emotionLab = self.builder.get_object("label1")
		self.moodLab = self.builder.get_object("label2")
		self.stateLab = self.builder.get_object("label3")
		self.leftCommandBox = self.builder.get_object("vbox2")
		self.rightCommandBox = self.builder.get_object("vbox3")
		self.buttonBox = self.builder.get_object("buttons")
		self.curCommandActions = self.builder.get_object("commandInfo")
		
		self.cameraL = self.builder.get_object("cameraL")
		#self.cameraL.set_from_file("triangle.png")
		self.cameraR = self.builder.get_object("cameraR")
		
		
		self.commandList = []
		
		GLib.timeout_add(100, self.update)

   

    def on_button_clicked(self, widget):
    	print ("button clicked")
    	command = widget.get_label()
    	text = "Chosen command: " + str(command) + "\n"
    	for c in self.rob.known_commands:
    		cText = c.camera + c.colour + c.shape
    		if cText == command:
    			text += c.actionsToPrint()
    		
    	
    	self.curCommandActions.set_text(text)
    	
    def loop(self):
    	while True:
    		self.rob.emotion+=1
    		print (self.rob.emotion)
        
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
    	#text = self.rob.last_command.actionsToPrint()
    	#self.curCommandActions.set_text(text)
    	text = "State: " + str(self.rob.state)
    	self.stateLab.set_text(text)
					
    	"""img.set_from_file("triangle.png")
    	self.leftCommandBox.pack_end(img, True, True, 10)
    	img = Gtk.Image() 
    	img.set_from_file("nothing.png")
    	self.rightCommandBox.pack_end(img, True, True, 10 )"""
    	
    	self.cameraL.set_from_file("curleft.jpg")
    	self.cameraR.set_from_file("curright.jpg")
    	
    	self.win.show_all()
    	return True # IMPORTANT
        
    
        

if __name__ == "__main__":
	"""win = MyWindow()
	win.connect("destroy", Gtk.main_quit)
	win.show_all()"""
	rospy.init_node("miro_ros_client_py", anonymous=True)
	main = MyWindow()
	#thread.start_new_thread(main.loop, ())
	Gtk.main()
	

	
	

