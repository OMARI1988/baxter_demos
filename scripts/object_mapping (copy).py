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
import colorsys

global img,x,y,c,flag1,flag2
img = []
x={}
y={}
c={}
flag1=0
flag2=0

fx = 525.0
fy = 525.0
cx = 319.5
cy = 239.5

th = 10
sp = 5

if __name__ == '__main__':

    br = CvBridge()	# Create a black image, a window


#--------------------------------------------------------------------------------------#
    def objects(imgmsg):
	global x,y,c,flag1
	x = {}
	y = {}
	c = {}
	for i in imgmsg.markers:
		data = len(i.points)
		x[i.id] = np.zeros(data,dtype=int)
		y[i.id] = np.zeros(data,dtype=int)
		c[i.id] = [int(i.color.b*255), int(i.color.g*255), int(i.color.r*255)]
		for j in range(data):
			#x_world = (x - cx) * z / fx
			#y_world = (y - cy) * z / fy
			k = i.points[j]
			x[i.id][j] = int(k.x * fx / k.z + cx)
			y[i.id][j] = int(k.y * fy / k.z + cy)
	flag1=1

#--------------------------------------------------------------------------------------#
    def detect_and_draw(imgmsg):
	global img,flag2
        img = br.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")
	flag2=1

#--------------------------------------------------------------------------------------#
    def talker():
	global img,x,y,c,flag1,flag2
	while not rospy.is_shutdown():
		if flag1==1 and flag2==1:
			img2 = img.copy()
			x2 = x.copy()
			y2 = y.copy()
			c2 = c.copy()
			#------- plot the image like rviz --------#
			img2.setflags(write=True)
			for i in x2:
				img2[y2[i],x2[i]] = c2[i]
			#k = cv2.waitKey(1) & 0xFF

			#------- find the object colors --------#
			b,g,r = cv2.split(img2)
			#h = np.zeros((300,3*th+4*sp*len(x2),3))
			h = np.zeros((300,int((3*th+4*sp)*len(x2)),3), np.uint8)+255
			counter = 0
			for i in x2:
				b_list = b[y2[i],x2[i]].tolist()
				b_counter = np.zeros(256).tolist()

				g_list = g[y2[i],x2[i]].tolist()
				g_counter = np.zeros(256).tolist()

				r_list = r[y2[i],x2[i]].tolist()
				r_counter = np.zeros(256).tolist()

				for j in range(256):
					b_counter[j] = b_list.count(j)
					g_counter[j] = g_list.count(j)
					r_counter[j] = r_list.count(j)

				b_val = b_counter.index(max(b_counter))
				g_val = g_counter.index(max(g_counter))
				r_val = r_counter.index(max(r_counter))

				sh = (3*th+4*sp)*counter
				h[300-b_val:300, sh+(counter+1)*sp : sh+(counter+1)*sp+th,:] = [255,0,0]
				h[300-g_val:300, sh+(counter+2)*sp+th : sh+(counter+2)*sp+2*th,:] = [0,255,0]
				h[300-r_val:300, sh+(counter+3)*sp+2*th : sh+(counter+3)*sp+3*th,:] = [0,0,255]
				counter += 1

				X2 = int(np.mean(x2[i]))
				Y2 = int(np.mean(y2[i]))

				hsv = colorsys.rgb_to_hsv(r_val/255.0,g_val/255.0,b_val/255.0)

				cv2.putText(img2,"obj.id %s" % (i), (X2,Y2), cv2.FONT_HERSHEY_SIMPLEX, .5, 255)
				cv2.putText(img2,"rgb [%s,%s,%s]" % (r_val,g_val,b_val), (X2,Y2+15), cv2.FONT_HERSHEY_SIMPLEX, .5, 255)
				cv2.putText(img2,"hsv [%s,%s,%s]" % (int(hsv[0]*360),hsv[1],hsv[2]), (X2,Y2+30), cv2.FONT_HERSHEY_SIMPLEX, .5, 255)

		    	cv2.imshow('RGB',img2)
			cv2.imshow('color histograms',h)

			"""
			for i in x2:
				#h = np.zeros((300,256,3))
				bins = np.arange(256).reshape(256,1)
				#color = [ (255,0,0),(0,255,0),(0,0,255) ]
				hist_item = cv2.calcHist([item],[0],None,[256],[0,255])
				for item,col in zip([b[y2[i],x2[i]],g[y2[i],x2[i]],r[y2[i],x2[i]]],color):
				    hist_item = cv2.calcHist([item],[0],None,[256],[0,255])
				    cv2.normalize(hist_item,hist_item,0,255,cv2.NORM_MINMAX)
				    hist=np.int32(np.around(hist_item))
				    pts = np.column_stack((bins,hist))
				    cv2.polylines(h,[pts],False,col)

				h=np.flipud(h)
				cv2.imshow('colorhist for object '+str(i),h)
			"""
			k = cv2.waitKey(1) & 0xFF

			flag1 = 0
			flag2 = 0


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



