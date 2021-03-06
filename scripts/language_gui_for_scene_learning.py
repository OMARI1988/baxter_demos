#! /usr/bin/env python2.7
# -*- coding: iso-8859-1 -*-

import rospy
import Tkinter
import numpy as np
import cv_bridge
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
	
#-----------------------------------------------------------------------------------------------------#
class simpleapp_tk(Tkinter.Tk):

    def __init__(self,parent):
        Tkinter.Tk.__init__(self,parent)
        self.parent = parent
        self.initialize()
        self.pub = rospy.Publisher('sentence', String, queue_size=1)
        self.pub3 = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)

    def initialize(self):
        self.grid()

        self.entryVariable = Tkinter.StringVar()
        self.entry = Tkinter.Entry(self,textvariable=self.entryVariable,width=130)
        self.entry.grid(column=0,row=0,sticky='EW')
        #self.entry.bind("<Return>", self.OnPressEnter)
        self.entryVariable.set(u"Enter sentence.")

        button = Tkinter.Button(self,text=u"Add sentence",command=self.OnButtonClick)
        button.grid(column=1,row=0)

        #button2 = Tkinter.Button(self,text=u"Finish",command=self.Finish)
        #button2.grid(column=2,row=0)

        #button3 = Tkinter.Button(self,text=u"Save",command=self.Save)
        #button3.grid(column=3,row=0)

        #button4 = Tkinter.Button(self,text=u"Load",command=self.Load)
        #button4.grid(column=4,row=0)

        #button4 = Tkinter.Button(self,text=u"Pick",command=self.Pick)
        #button4.grid(column=5,row=0)

        self.labelVariable = Tkinter.StringVar()
        label = Tkinter.Label(self,textvariable=self.labelVariable,
                              anchor="w",fg="white",bg="blue")
        label.grid(column=0,row=1,columnspan=4,sticky='EW')
        self.labelVariable.set(u"Ready !")

        self.grid_columnconfigure(0,weight=1)
        self.resizable(True,False)
        self.update()
        self.geometry(self.geometry())       
        self.entry.focus_set()
        self.entry.selection_range(0, Tkinter.END)

    def OnButtonClick(self):
	
	    sentence = self.entryVariable.get()
	    self.pub.publish(sentence)
	    self.send_image('happy')
	    self.send_image('default')

    def send_image(self,img_name):
	"""
	Send the image located at the specified path to the head
	display on Baxter.

	@param path: path to the image file to load and send
	"""
	dirr = '/home/omari/ros_ws/src/baxter_demos/share/images/'
	path = dirr+img_name+'.png'
	img = cv2.imread(path)
	msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
	self.pub3.publish(msg)
	rospy.sleep(1)
    # Sleep to allow for image to be published.



#-----------------------------------------------------------------------------------------------------#
def initial():
	rospy.init_node('Language_GUI')


#-----------------------------------------------------------------------------------------------------#
if __name__ == "__main__":
    app = simpleapp_tk(None)
    app.title('Language Interface')
    rospy.loginfo('Language interface running..')
    initial()
    app.mainloop()

