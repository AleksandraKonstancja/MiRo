#!/usr/bin/env python

import gi
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
		self.builder.add_from_file("ver1.glade")
		self.win = self.builder.get_object("window1")
		self.win.connect("destroy", Gtk.main_quit)
		self.win.show_all()
		
		self.rob = Robot()
		self.rob.last_command=Command("left")
		self.rob.known_commands = [Command("left"), Command("right"), Command("left")]
		
		self.emotionLab = self.builder.get_object("label1")
		self.moodLab = self.builder.get_object("label2")
		self.stateLab = self.builder.get_object("label3")
		self.leftCommandBox = self.builder.get_object("vbox2")
		self.rightCommandBox = self.builder.get_object("vbox3")
		self.curCommandActions = self.builder.get_object("label5")
		
		self.commandList = []
		
		GLib.timeout_add(100, self.update)

   

    def on_button_clicked(self, widget):
        print("Hello World")
        
    def update(self):

    	text = "Emotion: " + str(self.rob.emotion) + " / 1"
    	self.emotionLab.set_text(text)
    	text = "Mood: " + str(self.rob.mood) + " / 1"
    	self.moodLab.set_text(text)
    	if len(self.rob.known_commands) > len(self.commandList):
    		self.commandList.append(self.rob.known_commands[-1])
    		img = Gtk.Image()
    		img.set_from_file("triangle.png")
    		img2 = Gtk.Image()
    		img2.set_from_file("nothing.png")
    		if self.commandList[-1].camera == "left":
    			self.leftCommandBox.pack_end(img, True, True, 10)
    			self.rightCommandBox.pack_end(img2, True, True, 10 )
    		elif self.commandList[-1].camera == "right":
    			self.leftCommandBox.pack_end(img2, True, True, 10)
    			self.rightCommandBox.pack_end(img, True, True, 10 )
    			
    	text = self.rob.last_command.actionsToPrint()
    	self.curCommandActions.set_text(text)
					
    	"""img.set_from_file("triangle.png")
    	self.leftCommandBox.pack_end(img, True, True, 10)
    	img = Gtk.Image() 
    	img.set_from_file("nothing.png")
    	self.rightCommandBox.pack_end(img, True, True, 10 )"""
    	
    	self.win.show_all()
    	return True # IMPORTANT
        
    
        

if __name__ == "__main__":
	"""win = MyWindow()
	win.connect("destroy", Gtk.main_quit)
	win.show_all()"""
	main = MyWindow()
	Gtk.main()

	
	

