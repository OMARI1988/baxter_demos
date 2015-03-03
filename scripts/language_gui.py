#! /usr/bin/env python2.7
# -*- coding: iso-8859-1 -*-

import rospy
import Tkinter
import numpy as np
from networkx import *
#-----------------------------------------------------------------------------------------------------#
global f

f = open('/home/omari/ros_ws/src/baxter_demos/share/language.txt', 'w')

#-----------------------------------------------------------------------------------------------------#

#-------------------------------------------------------------------------------------#
def callback_graphs(data):
	A = data

#-----------------------------------------------------------------------------------------------------#
class App():
   def __init__(self):
       self.root = Tkinter.Tk()
       button = Tkinter.Button(self.root, text = 'root quit', command = self.quit)# I know this way: command = self.root.destroy
       button.pack()
       self.root.mainloop()
   def quit(self):
       self.root.destroy #This way is not work, why? Can you make this
#-----------------------------------------------------------------------------------------------------#
def listener():
    rospy.init_node('Language_GUI')

#-----------------------------------------------------------------------------------------------------#
if __name__ == "__main__":
    global A
    A = []
    app = App()
    app.title('Language Interface')

    print('Language interface running...  ')

    listener()
    app.mainloop()

