#!/usr/bin/env python

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk

class MyWindow(Gtk.Window):

    def __init__(self):
        """Gtk.Window.__init__(self, title="Hello World")
        
        vboxMain = Gtk.Box(orientation=Gtk.Orientation.VERTICAL,spacing = 10)
        
        hboxState = Gtk.Box(spacing = 10)
        vboxMood = Gtk.Box(spacing = 10)
        vboxEmotion = Gtk.Box(spacing = 10)
        vboxState = Gtk.Box(spacing=10)
        label1 = Gtk.Label("Mood: ")
        #vboxMood.pack_end(label, True, True, 0 )
        label2 = Gtk.Label("Emotion: ")
        #vboxEmotion.pack_end(label, True, True, 0)
        label3 = Gtk.Label("State: ")
        #vboxEmotion.pack_end(label, True, True, 0)
        #hboxState.pack_start(label1, True, True, 0)
        #hboxState.pack_start(label2, True, True, 0)
        #hboxState.pack_start(label3, True, True, 0)
        
        hboxCommands = Gtk.Box(spacing = 10)
        label = Gtk.Label("Known Commands")"""
        
        """#hboxCommands.pack_start(label, True, True, 0 )
        
        vboxCommands = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing = 10)
        img = Gtk.Image()
        img.set_from_file("img1.jpg")
        vboxCommands.pack_start(img, True, True, 0 )
        img= Gtk.Image()
        img.set_from_file("img4.jpg")
        vboxCommands.pack_start(img, True, True, 0 )
        
        align = Gtk.Alignment(xalign = 0.5,yalign = 0, xscale = 1, yscale= 1)
        vboxActions = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing = 5)
        label1 = Gtk.Label("Go: 90, turnL: 5, turnR: 3, ... \nGo: 20, turnL: 45, turnR: 15, ...")
        vboxActions.pack_start(label1, True, True, 0)
        align.add(vboxActions)

        
        hboxCommands.pack_start(vboxCommands, True, True, 0)
        hboxCommands.pack_start(align, True, True, 0)
        
        vboxMain.pack_start(hboxState,False, False, 0)
        vboxMain.pack_start(label, True, True, 0)
        vboxMain.pack_start(hboxCommands, True, True, 0 )"""
        
        
        """grid = Gtk.Grid()
        grid.set_column_spacing(10)
        grid.set_row_spacing(10)
        grid.add(label1)
        grid.add(label2)
        grid.add(label3)
        grid.attach(label, 0,1,3,1)
        img = Gtk.Image()
        img.set_pixel_size(5000)
        img.set_from_file("img1.jpg")
        grid.attach(img, 0,2,1,1)
        img = Gtk.Image()
        img.set_from_file("img4.jpg")
        grid.attach(img, 0,3,1,1)
        label = Gtk.Label("Go: 90, turnL: 5, turnR: 3, ... \nGo: 20, turnL: 45, turnR: 15, ...")
        grid.attach(label, 1,2,2,1)"""
        
        
        


        """self.button = Gtk.Button(label="Click Here")
        self.button.connect("clicked", self.on_button_clicked)
        #self.add(self.button)
        self.add(grid)"""
        

    def on_button_clicked(self, widget):
        print("Hello World")

if __name__ == "__main__":
	"""win = MyWindow()
	win.connect("destroy", Gtk.main_quit)
	win.show_all()"""
	builder = Gtk.Builder()
	builder.add_from_file("ver1.glade")
	win = builder.get_object("window1")
	win.connect("destroy", Gtk.main_quit)
	win.show_all()
	Gtk.main()

