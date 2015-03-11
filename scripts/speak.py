#!/usr/bin/env python

import sys
import rospy
from ros_mary_tts.srv import *

def speak(x):
    rospy.wait_for_service('ros_mary')
    try:
        add_two_ints = rospy.ServiceProxy('ros_mary',ros_mary)
        resp1 = add_two_ints(x)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
	speak('Hello! Welcome to the school of computing, my name is Lucas. Which stands for. Leeds university cognative artificial system. Researchers here in leeds are teaching me how to become a smarter robot! so that I can help humans in their daily activities! One of the many interesting things I can do is, you can ask me to pick up an object and I will pick it up for you! Please try and ask me to pick something!')
	#speak('Ready to work!, sir.')
