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
import cv2.cv as cv
import numpy as np
#import scipy.ndimage.morphology as morphology
from baxter_demos.msg import obj_relations,obj_hypotheses
import visualization_msgs.msg
from geometry_msgs.msg import Pose,Point
import colorsys
#from geometry_msgs.msg import Pose,Point


import numpy as np
import cv2

global x,y
x={}
y={}

if __name__ == '__main__':

    br = CvBridge()	# Create a black image, a window

#--------------------------------------------------------------------------------------#
    def detect_and_draw(imgmsg):
        global x,y
        img = br.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")
        img = img[:,:,0:3]
        #cv2.imshow('inverse',cv2.bitwise_not(img))
        #img = cv2.bitwise_not(img)
        img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        #img = img[:,:,0]

        img = cv2.medianBlur(img,5)
        #print img[img>0]
        cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
			
        circles = cv2.HoughCircles(img,cv.CV_HOUGH_GRADIENT,1,2,param1=100,param2=30,minRadius=10,maxRadius=50)

        circles = np.uint16(np.around(circles))
        centers = []
        for i in circles[0,:]:
            # draw the outer circle
            #print i[2]
            cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
            centers.append([i[0],i[1],i[2]])
        
        dis_TR = 100000
        dis_TL = 100000
        
        for i in centers:
            dis = np.sqrt((i[0]-1000)**2 + (i[1])**2)
            if dis < dis_TR:
                dis_TR = dis
                TR = i
            dis = np.sqrt((i[0])**2 + (i[1])**2)
            if dis < dis_TL:
                dis_TL = dis
                TL = i

        TR = map(int,TR)
        TL = map(int,TL)
        print img.shape
        #cv2.circle(cimg,(480,300),2,(0,0,0),3)
        # draw TR
        #cv2.circle(cimg,(TR[0],TR[1]),TR[2],(0,255,0),2)
        #cv2.circle(cimg,(TR[0],TR[1]),2,(0,0,255),3)
        #cv2.circle(cimg,(TL[0],TL[1]),TL[2],(0,0,255),2)
        #cv2.circle(cimg,(TL[0],TL[1]),2,(0,0,255),3)
        #print TR,TL,(TR[0]-TL[0])**2,TR[1]-TL[1]
        #dis = np.sqrt((TR[0]-TL[0])**2 + (TR[1]-TL[1])**2)
        #print dis/5
        dx = (float(TR[0])-float(TL[0]))/4
        dy = (float(TR[1])-float(TL[1]))/4
        #for i in range(1,4):
        #    cv2.circle(cimg,(TL[0]+int(np.round(dx*i)),TL[1]+int(np.round(dy*i))),2,(0,0,255),3)
        #    print 'dx:',480-(TL[0]+int(np.round(dx*i))),'dy:',300-(TL[1]+int(np.round(dy*i)))
        #    print (TL[0]+int(np.round(dx*i)))
            
        #print '--------------'
        #print dx,dy
        

        cv2.imshow('detected circles',cimg)
        cv2.waitKey(1)




#--------------------------------------------------------------------------------------#


    fgbg = cv2.BackgroundSubtractorMOG()
    rospy.init_node('object_detection')
    LH_image_topic = rospy.resolve_name("/cameras/left_hand_camera/image")

    rospy.Subscriber(LH_image_topic, sensor_msgs.msg.Image, detect_and_draw)

    rospy.spin()



