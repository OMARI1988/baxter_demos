#! /usr/bin/env python2.7

import roslib
roslib.load_manifest('baxter_demos')
import sys
import os
from optparse import OptionParser
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
#import cv
import cv2
import numpy as np
#import scipy.ndimage.morphology as morphology

from std_msgs.msg import String
import visualization_msgs.msg

global img,x,y
img = []
x = []

fx = 525.0
fy = 525.0
cx = 319.5
cy = 239.5

if __name__ == '__main__':


    #pkgdir = roslib.packages.get_pkg_dir("opencv2")
    #haarfile = os.path.join(pkgdir, "opencv/share/opencv/haarcascades/haarcascade_frontalface_alt.xml")
    br = CvBridge()	# Create a black image, a window


#--------------------------------------------------------------------------------------#
    def objects(imgmsg):
	global x,y
	x = {}
	y = {}
	for i in imgmsg.markers:
		print i.id
		data = len(i.points)
		x[i.id] = np.zeros(data,dtype=int)
		y[i.id] = np.zeros(data,dtype=int)
		for j in range(data):
			#x_world = (x - cx) * z / fx
			#y_world = (y - cy) * z / fy
			k = i.points[j]
			x[i.id][j] = int(k.x * fx / k.z + cx)
			y[i.id][j] = int(k.y * fy / k.z + cy)

#--------------------------------------------------------------------------------------#
    def detect_and_draw(imgmsg):
	global img
        img = br.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")

#--------------------------------------------------------------------------------------#
    def talker():
	global img,x,y
	while not rospy.is_shutdown():
		if img != [] and x!=[]:
			img2 = img
			img2.setflags(write=True)
			for i in x:
				x2 = x[i]
				y2 = y[i]
				#print img2[x2[0],y2[0]]
				img2[y2,x2]=[255,255,255]
		    	cv2.imshow('RGB',img2)
			k = cv2.waitKey(1) & 0xFF


#--------------------------------------------------------------------------------------#


    rospy.init_node('rosColordetect')
    image_topic = rospy.resolve_name("/camera/rgb/image_color") 
    depth_topic = rospy.resolve_name("/camera/depth_registered/image_raw") 
    object_topic = rospy.resolve_name("/object_recognition_2/tabletop/clusters") 

    

    rospy.Subscriber(image_topic, sensor_msgs.msg.Image, detect_and_draw)
    rospy.Subscriber(object_topic, visualization_msgs.msg.MarkerArray, objects)
    #rospy.Subscriber(depth_topic, sensor_msgs.msg.Image, depth_calculation)
    talker()
    print 'test'

    rospy.spin()



