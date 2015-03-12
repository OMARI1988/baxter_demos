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
	
	if flag1 == 0:
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
		flag1=1

#--------------------------------------------------------------------------------------#
    def hypotheses(data):
	global hyp
	hyp = data


#--------------------------------------------------------------------------------------#
    def detect_and_draw(imgmsg):
	global img,flag2
        img = br.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")
	flag2=1


#--------------------------------------------------------------------------------------#
    def talker():
	global img,x,y,c,flag1,flag2,xyz,hyp
	while not rospy.is_shutdown():
		if flag1==1 and flag2==1:
			img2 = img.copy()
			x2 = x.copy()
			y2 = y.copy()
			c2 = c.copy()
			xyz2 = xyz.copy()

			#------- find the object colors --------#
			b,g,r = cv2.split(img2)
			#h = np.zeros((300,int((3*th+4*sp)*len(x2)),3), np.uint8)+255
			#counter = 0
			ID = []
			H = []
			S = []
			V = []
			X_img = []
			Y_img = []
			X = []			#center of object in robot frame x
			Y = []			#center of object in robot frame y
			Z = []			#center of object in robot frame z
			yaw = []
			distance = []
			direction = []
			for i in x2:
				ID.append(i)
				b_val = int(np.mean(b[y2[i],x2[i]].tolist()))
				g_val = int(np.mean(g[y2[i],x2[i]].tolist()))
				r_val = int(np.mean(r[y2[i],x2[i]].tolist()))

				X2 = int(np.mean(x2[i]))
				Y2 = int(np.mean(y2[i]))

				X_img.append(X2)
				Y_img.append(Y2)

				hsv = colorsys.rgb_to_hsv(r_val/255.0,g_val/255.0,b_val/255.0)
				H.append(int(hsv[0]*360))
				S.append(hsv[1])
				V.append(hsv[2])

				cv2.putText(img2,"obj.id %s" % (i), (X2,Y2), cv2.FONT_HERSHEY_SIMPLEX, .5, 0)
				cv2.putText(img2,"rgb [%s,%s,%s]" % (r_val,g_val,b_val), (X2,Y2+15), cv2.FONT_HERSHEY_SIMPLEX, .5, 0)
				cv2.putText(img2,"hsv [%s,%1.2f,%1.2f]" % (int(hsv[0]*360),hsv[1],hsv[2]), (X2,Y2+30), cv2.FONT_HERSHEY_SIMPLEX, .5, 0)

			counter = 1

			for i in range(len(xyz2)):
				

				p = Point()
				p.x = np.mean(xyz2[i]['x'])
				p.y = np.mean(xyz2[i]['y'])
				p.z = np.mean(xyz2[i]['z'])			
				K = camera_to_torso(p)
				X.append(K.x)
				Y.append(K.y)
				Z.append(K.z)
				yaw.append(0.0) 			# to be done later

			for i in range(len(X)-1):
				for j in range(i+1,len(X)):
					
					dis = np.sqrt(  (np.abs(X[i]-X[j]))**2  +  (np.abs(Y[i]-Y[j]))**2  +  (np.abs(Z[i]-Z[j]))**2  )
					dirr = direction_calc([X[i],Y[i]],[X[j],Y[j]])
					cv2.putText(img2,"relations [%s,%s] = (d,%4.2f), (a,%4.2f)" % (i,j,dis,dirr*180/np.pi), (10,480-20*counter), cv2.FONT_HERSHEY_SIMPLEX, .5, 0)
					counter+=1
					distance.append(dis)
					direction.append(dirr)

			if hyp != []:
			    if len(hyp.obj) == len(hyp.Ximg):
				for i in range(len(hyp.obj)):
					X2 = hyp.Ximg[i]
					Y2 = hyp.Yimg[i]
					cv2.putText(img2,"hyp = %s" % (hyp.obj[i]), (X2,Y2+45), cv2.FONT_HERSHEY_SIMPLEX, .5, 0)

				for counter in range(len(hyp.distance)):
					cv2.putText(img2,"(d,%s), (a,%s)" % (hyp.distance[counter],hyp.direction[counter]), (350,480-20*(counter+1)), cv2.FONT_HERSHEY_SIMPLEX, .5, 0)
						
			pub.publish(ID,distance,direction,H,S,V,X_img,Y_img,X,Y,Z,yaw)

			#print xi ,yi ,zi


			#------- plot the image like rviz --------#
			#img2.setflags(write=True)
			#for i in x2:
			#	img2[y2[i],x2[i]] = c2[i]
			#k = cv2.waitKey(1) & 0xFF

		    	cv2.imshow('RGB',img2)
			#cv2.imshow('color histograms',h)

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
			flag1 = 0
			flag2 = 0
			k = cv2.waitKey(1) & 0xFF



#--------------------------------------------------------------------------------------#


    rospy.init_node('object_relations')
    rospy.loginfo('Object relations running..')
    image_topic = rospy.resolve_name("/camera/rgb/image_color") 
    object_topic = rospy.resolve_name("/object_recognition_2/tabletop/clusters") 

    rospy.Subscriber(image_topic, sensor_msgs.msg.Image, detect_and_draw)
    rospy.Subscriber(object_topic, visualization_msgs.msg.MarkerArray, objects)

    pub = rospy.Publisher('obj_relations', obj_relations, queue_size=1)
    rospy.Subscriber('/obj_hypotheses', obj_hypotheses, hypotheses)
    talker()

    rospy.spin()



