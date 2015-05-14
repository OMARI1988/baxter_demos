#! /usr/bin/env python2.7

import roslib
roslib.load_manifest('baxter_demos')
import rospy
import sensor_msgs.msg
import visualization_msgs.msg
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy import ndimage
from skimage import data, io, segmentation, color
from skimage.future import graph
from skimage.feature import hog
from skimage import data, color, exposure, io, measure, filters, transform
from skimage.feature import canny, CENSURE
from skimage.measure import label, regionprops
import math
"""
am using the sobel edge detector

"""

#--------------------------------------------------------------------------------------#
class scene_segment():

    def _weight_mean_color(self,graph, src, dst, n):
        diff = graph.node[dst]['mean color'] - graph.node[n]['mean color']
        diff = np.linalg.norm(diff)
        return diff

    def merge_mean_color(self,graph, src, dst):
        graph.node[dst]['total color'] += graph.node[src]['total color']
        graph.node[dst]['pixel count'] += graph.node[src]['pixel count']
        graph.node[dst]['mean color'] = (graph.node[dst]['total color'] /
                                         graph.node[dst]['pixel count'])

    def segment_image(self,img,name):
        labels = segmentation.slic(img, compactness=30, n_segments=200)
        g = graph.rag_mean_color(img, labels)
        labels2 = graph.merge_hierarchical(labels, g, thresh=80, rag_copy=False,
                                       in_place_merge=True,
                                       merge_func=self.merge_mean_color,
                                       weight_func=self._weight_mean_color)
        self.result = color.label2rgb(labels2, img, kind='avg')
        cv2.imshow('segmentation : '+str(name), self.result)
        k = cv2.waitKey(1) & 0xFF
        return self.result
        
            
#--------------------------------------------------------------------------------------#
class object_detection():
    def __init__(self):

        self.enlarge = 1.5
        self.cv_bridge = CvBridge()	                # initilize opencv
        self.x = {}
        self.y = {}
        self.x_use = {}
        self.y_use = {}
        #self.xyz = {}
        self.fx = 525.0
        self.fy = 525.0
        self.cx = 319.5
        self.cy = 239.5
        self.idd = {}
        self.flag = 0
        self.main_counter = 0
        self.frame_counter = 0
        self.Img = []
        self.N = 0

        xtion_rgb_topic = rospy.resolve_name("/camera/rgb/image_color") 
        rospy.Subscriber(xtion_rgb_topic, sensor_msgs.msg.Image, self._xtion_rgb)

        object_topic = rospy.resolve_name("/object_recognition_2/tabletop/clusters")
        rospy.Subscriber(object_topic, visualization_msgs.msg.MarkerArray, self._clusters)
        
        self.segment = scene_segment()
        
    def _clusters(self,cluster):
        if self.flag != 1:
            print len(cluster.markers)
            for i in cluster.markers:
                data = len(i.points)
                self.x[i.id] = np.zeros(data,dtype=int)
                self.y[i.id] = np.zeros(data,dtype=int)
                #self.xyz[i.id] = {}
                #self.xyz[i.id]['x'] = np.zeros(data,dtype=float)
                #self.xyz[i.id]['y'] = np.zeros(data,dtype=float)
                #self.xyz[i.id]['z'] = np.zeros(data,dtype=float)
                for j in range(data):
                    k = i.points[j]
                    #self.xyz[i.id]['x'][j] = k.x
                    #self.xyz[i.id]['y'][j] = k.y
                    #self.xyz[i.id]['z'][j] = k.z
                    #xyz[i.id]
                    self.x[i.id][j] = int(k.x * self.fx / k.z + self.cx)
                    self.y[i.id][j] = int(k.y * self.fy / k.z + self.cy)

                self.idd[i.id] = 0
            self.flag = 1

            for j in self.x.keys():
                self.idd[j]+=1
                if self.idd[j] == 5:
                    self.idd.pop(j, None)
                    self.x.pop(j, None)
                    self.y.pop(j, None)
                    #self.xyz.pop(j,None)

    def _xtion_rgb(self,imgmsg):
        self.xtion_img_rgb = self.cv_bridge.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough") 
        self.xtion_img_rgb.setflags(write=True)                   # allow to change the values 
        self.xtion_img_rgb_original = self.xtion_img_rgb.copy()

        self._plot_results()    
        #cv2.imshow('xtion rgb',self.xtion_img_rgb_original)
        #cv2.imshow('xtion rgb clustered',self.xtion_img_rgb)
        k = cv2.waitKey(1)
        
    def _plot_results(self):
        if self.flag != 0:
            self.x_use = self.x.copy()
            self.y_use = self.y.copy()
            self.flag = 0

        #filtere_img = np.zeros(self.xtion_img_rgb.shape, dtype=np.uint8)
        #canny_img = np.zeros(self.xtion_img_rgb.shape, dtype=np.uint8)
        for key in self.x_use:
            x1 = min( self.x_use[key])
            x2 = max( self.x_use[key])
            y1 = min( self.y_use[key])
            y2 = max( self.y_use[key])


            test_img = np.zeros(self.xtion_img_rgb.shape, dtype=np.uint8)
            if np.mod(self.frame_counter,10)==0:
                xc = np.mean(self.x_use[key])
                yc = np.mean(self.y_use[key])
                X = np.asarray(self.x_use[key])
                Y = np.asarray(self.y_use[key])

                test_img[Y,X,:] = [255,255,255]

                imgray = cv2.cvtColor(test_img,cv2.COLOR_BGR2GRAY)
                ret,thresh = cv2.threshold(imgray,127,255,0)
                contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(test_img, contours, -1, (255,255,255), 10)

                label_img = label(test_img[:,:,0])
                regions = regionprops(label_img)
                for props in regions:
                    y0, x0 = props.centroid
                    x0 = int(np.round(x0))
                    y0 = int(np.round(y0))
                    orientation = props.orientation
                    x1 = int(np.round(x0 + math.cos(orientation) * 0.5 * props.major_axis_length))
                    y1 = int(np.round(y0 - math.sin(orientation) * 0.5 * props.major_axis_length))
                    x2 = int(np.round(x0 - math.sin(orientation) * 0.5 * props.minor_axis_length))
                    y2 = int(np.round(y0 - math.cos(orientation) * 0.5 * props.minor_axis_length))
                    cv2.line(test_img,(x0,y0),(x1,y1),(0,0,255),1)
                    cv2.line(test_img,(x0,y0),(x2,y2),(255,0,0),1)
                    #cv2.line(test_img,(x1,y2),(x2,y2),(0,0,255),1)
                #cv2.imshow('test image',test_img)
        
                if orientation<0:   ang = -90-orientation*180/np.pi
                if orientation>=0:  ang = 90-orientation*180/np.pi

                img = transform.rotate(self.xtion_img_rgb_original,ang,resize=False, center=(xc,yc), order=1, mode='constant', cval=0, clip=True, preserve_range=False)
                img = img[yc-80:yc+80,xc-80:xc+80]
                if img.shape[0] == 160 and img.shape[1] == 160:
                    imgray = color.rgb2gray(img)
                    filtered = filters.sobel(imgray)  
                    filtered[filtered>0.1] = 1.0
                    if self.N == 0:     self.Img = filtered
                    else:               self.Img += filtered
                    self.N += 1

                    Hist,Img = hog(self.Img/self.N, orientations=8, pixels_per_cell=(16, 16), cells_per_block=(1, 1), visualise=True)
                    #filtere_img[yc-80:yc+80,xc-80:xc+80,:] = (color.gray2rgb(filtered)*255).astype(int)
                    #canny_img[yc-80:yc+80,xc-80:xc+80,:] = color.gray2rgb(canny_i)
                    #cv2.imshow('filtered image',filtere_img)
                    #cv2.imshow('test image',test_img)
                    #cv2.imshow('canny',canny_img)
                    cv2.imshow('img',img)
                    cv2.imshow('filtered',color.gray2rgb(filtered))
                    cv2.imshow('histogram',Img)
                    cv2.imshow('histogram2',self.Img/self.N)
            cv2.rectangle(self.xtion_img_rgb, (x1,y1), (x2,y2), (255*np.random.rand(3)).astype(int), 2)
        self.frame_counter += 1
            #cv2.imshow('filtered image',filtere_img)

#--------------------------------------------------------------------------------------#
def main():
    object_detection()
    
    rospy.init_node('object_detection')
    rospy.loginfo('Object detection running..')
    rospy.spin()
    
#--------------------------------------------------------------------------------------#
if __name__ == '__main__':
    main()







