#! /usr/bin/env python2.7

import roslib
roslib.load_manifest('baxter_demos')
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy import ndimage

#--------------------------------------------------------------------------------------#
class object_detection():
    def __init__(self):
        self.cv_bridge = CvBridge()	                # initilize opencv
        self.BS1 = cv2.BackgroundSubtractorMOG()    #background subtraction for xtion RGB
        
        xtion_rgb_topic = rospy.resolve_name("/camera/rgb/image_color") 
        rospy.Subscriber(xtion_rgb_topic, sensor_msgs.msg.Image, self._xtion_rgb)
        
        
    def _xtion_rgb(self,imgmsg):
        self.xtion_img = self.cv_bridge.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")
        self.xtion_mask = self._subtract(self.xtion_img.copy(), self.BS1)
        self.xtion_result = self._plot_results(self.xtion_img.copy(), self.xtion_mask)
        
        cv2.imshow('xtion rgb',self.xtion_img)
        cv2.imshow('xtion rgb masked',self.xtion_result)
        k = cv2.waitKey(1) & 0xff
        
    def _subtract(self, img, BS):
        fgmask = BS.apply(img)
        contour,hier = cv2.findContours(fgmask,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contour:
            cv2.drawContours(fgmask,[cnt],0,255,-1)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(15,15))
        fgmask = cv2.morphologyEx(fgmask,cv2.MORPH_OPEN,kernel)
        fgmask = cv2.bitwise_not(fgmask)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(30,30))
        fgmask = cv2.morphologyEx(fgmask,cv2.MORPH_OPEN,kernel)
        fgmask = cv2.bitwise_not(fgmask)
        return fgmask
        
    def _plot_results(self, img, fgmask):
        contour,hier = cv2.findContours(fgmask,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
        for count,cnt in enumerate(contour):
            area = cv2.contourArea(cnt)
            x,y,w,h = cv2.boundingRect(cnt)
            cv2.rectangle(img, (x,y), (x+w,y+h), (255*np.random.rand(3)).astype(int), 2)
            #roi=img[y:y+h,x:x+w]
            #cv2.imshow(str(count),roi)
        return img
            



#--------------------------------------------------------------------------------------#
def main():
    object_detection()
    
    rospy.init_node('object_detection')
    rospy.loginfo('Object detection running..')
    rospy.spin()
    
#--------------------------------------------------------------------------------------#
if __name__ == '__main__':
    main()







