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
from baxter_demos.msg import obj_relations,obj_hypotheses
import visualization_msgs.msg
from geometry_msgs.msg import Pose,Point
import colorsys
#from geometry_msgs.msg import Pose,Point


import numpy as np
import cv2

cap = cv2.VideoCapture('vtest.avi')

fgbg = cv2.BackgroundSubtractorMOG()
fgbg2 = cv2.BackgroundSubtractorMOG()
fgbg3 = cv2.BackgroundSubtractorMOG()



cap.release()
cv2.destroyAllWindows()


global img,x,y,c,flag1,flag2,hyp
hyp=[]
img = []
x={}
y={}
c={}
xyz = {}
idd={}
flag1=0
flag2=0

fx = 525.0
fy = 525.0
cx = 319.5
cy = 239.5

th = 10
sp = 5

rx = 0.0
ry = 0.7854
rz = 0.0
cosy = np.cos(ry)
siny = np.sin(ry)

dx = 0.22
dy = 0.08
dz = 0.18

if __name__ == '__main__':

    br = CvBridge()	# Create a black image, a window



#--------------------------------------------------------------------------------------#
    def camera_to_torso(p):

	# rotate frames around y axis and then offset them to match the kinect position on the robot torso
	P = Point()
	P.x = p.z 
	P.y = -(p.x+.042)
	P.z = -p.y 
	
	x = cosy*P.x + siny*P.z
	z = -siny*P.x + cosy*P.z
	
	P.x = x + dx
	P.y = P.y + dy
	P.z = z + dz
	return P

#--------------------------------------------------------------------------------------#
    def direction_calc(A,B):
	X = A[0]-B[0]
	Y = A[1]-B[1]
	C = np.arctan2(Y,X)
	if C<0:
		C=C+np.pi
	#int(C*100)/100.0
	return C
#--------------------------------------------------------------------------------------#
    def objects(imgmsg):
	global xyz,x,y,c,flag1,idd
	
	for i in imgmsg.markers:
		data = len(i.points)
		x[i.id] = np.zeros(data,dtype=int)
		y[i.id] = np.zeros(data,dtype=int)
		c[i.id] = [int(i.color.b*255), int(i.color.g*255), int(i.color.r*255)]
		xyz[i.id] = {}
		xyz[i.id]['x'] = np.zeros(data,dtype=float)
		xyz[i.id]['y'] = np.zeros(data,dtype=float)
		xyz[i.id]['z'] = np.zeros(data,dtype=float)
		for j in range(data):
			k = i.points[j]
			xyz[i.id]['x'][j] = k.x
			xyz[i.id]['y'][j] = k.y
			xyz[i.id]['z'][j] = k.z
			#xyz[i.id]
			x[i.id][j] = int(k.x * fx / k.z + cx)
			y[i.id][j] = int(k.y * fy / k.z + cy)

		idd[i.id] = 0

	for j in x.keys():
		idd[j]+=1
		if idd[j] == 5:
			idd.pop(j, None)
			x.pop(j, None)
			y.pop(j, None)
			c.pop(j, None)
			xyz.pop(j,None)

#--------------------------------------------------------------------------------------#
    def detect_and_draw(imgmsg):
	global x,y

        img = br.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")
	img = img[:,:,0:3]
	fgmask = fgbg.apply(img)

	cv2.imshow('rgb',img)
	#cv2.imshow('frame',fgmask)
	img1_bg = cv2.bitwise_and(img,img,mask = fgmask)
	img_obj = img.copy()
	for i in range(len(x)):
		print i
		img_obj[y[i],x[i],:] = [255,0,0]
	cv2.imshow('object',img_obj)
	cv2.imshow('masked',img1_bg)
	k = cv2.waitKey(1) & 0xff

#--------------------------------------------------------------------------------------#
    def detect_and_draw_depth(imgmsg):
	global x,y,z,rgb_flag

        depth = br.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")
	rgb_flag =0
	if rgb_flag == 1:
	    for i in range(len(x)):
		z[i] = float(depth[y[i],x[i]])
		rgb_flag = 0

    	#cv2.imshow('Depth',depth)
	k = cv2.waitKey(1) & 0xff

#--------------------------------------------------------------------------------------#
    def RH_image(imgmsg):
        img = br.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")
    	cv2.imshow('RH_RGB',img)

#--------------------------------------------------------------------------------------#
    def LH_image(imgmsg):
        img = br.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")
	img = img[:,:,0:3]
	fgmask2 = fgbg2.apply(img)

	cv2.imshow('rgb2',img)
	cv2.imshow('frame2',fgmask2)
	img1_bg = cv2.bitwise_and(img,img,mask = fgmask2)
	cv2.imshow('masked2',img1_bg)
	k = cv2.waitKey(1) & 0xff

#--------------------------------------------------------------------------------------#


    rospy.init_node('object_detection')
    rospy.loginfo('Object relations running..')
    image_topic = rospy.resolve_name("/camera/rgb/image_color") 
    depth_topic = '/camera/depth_registered/hw_registered/image_rect_raw'
    RH_image_topic = rospy.resolve_name("/cameras/right_hand_camera/image")
    LH_image_topic = rospy.resolve_name("/cameras/left_hand_camera/image")
    object_topic = rospy.resolve_name("/object_recognition_2/tabletop/clusters") 

    rospy.Subscriber(image_topic, sensor_msgs.msg.Image, detect_and_draw)
    rospy.Subscriber(depth_topic, sensor_msgs.msg.Image, detect_and_draw_depth)
    #rospy.Subscriber(RH_image_topic, sensor_msgs.msg.Image, RH_image)
    #rospy.Subscriber(LH_image_topic, sensor_msgs.msg.Image, LH_image)
    rospy.Subscriber(object_topic, visualization_msgs.msg.MarkerArray, objects)

    #pub = rospy.Publisher('obj_relations', obj_relations, queue_size=1)
    #rospy.Subscriber('/obj_hypotheses', obj_hypotheses, hypotheses)
    #talker()

    rospy.spin()



