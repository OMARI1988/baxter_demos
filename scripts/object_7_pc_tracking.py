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
        self.idd = {}
        #self.xyz = {}
        self.histograms = {}
        self.fx = 525.0
        self.fy = 525.0
        self.cx = 319.5
        self.cy = 239.5
        self.flag = 0
        self.frame_counter = 0

        xtion_rgb_topic = rospy.resolve_name("/camera/rgb/image_color") 
        rospy.Subscriber(xtion_rgb_topic, sensor_msgs.msg.Image, self._xtion_rgb)

        object_topic = rospy.resolve_name("/object_recognition_2/tabletop/clusters")
        rospy.Subscriber(object_topic, visualization_msgs.msg.MarkerArray, self._clusters)
        
        self.segment = scene_segment()
        
    def _clusters(self,cluster):  
        k = cv2.waitKey(1)
        if self.flag != 1:
            # parsing Data
            x = {}
            y = {} 
            mean = {} 
            for key in cluster.markers:
                data = len(key.points)
                x[key.id] = np.zeros(data,dtype=int)
                y[key.id] = np.zeros(data,dtype=int)
                #self.xyz[i.id] = {}
                #self.xyz[i.id]['x'] = np.zeros(data,dtype=float)
                #self.xyz[i.id]['y'] = np.zeros(data,dtype=float)
                #self.xyz[i.id]['z'] = np.zeros(data,dtype=float)
                for j in range(data):
                    k = key.points[j]
                    #self.xyz[i.id]['x'][j] = k.x
                    #self.xyz[i.id]['y'][j] = k.y
                    #self.xyz[i.id]['z'][j] = k.z
                    #xyz[i.id]
                    x[key.id][j] = int(k.x * self.fx / k.z + self.cx)
                    y[key.id][j] = int(k.y * self.fy / k.z + self.cy)

                mean[key.id] = [int(np.mean(x[key.id])), int(np.mean(y[key.id]))]

            # update tracks
            if self.x != {}:
                # measure distance between means
                matches = []
                for i in self.mean:
                    for j in mean:
                        dist = int(np.sqrt((self.mean[i][0] - mean[j][0])**2 + (self.mean[i][1] - mean[j][1])**2))
                        # do the update
                        if dist < 15:
                            matches.append(j)
                            self.x[i] = x[j]
                            self.y[i] = y[j]
                            self.mean[i] = mean[j]
                            self.idd[i] = 0

                # add new objects
                if len(matches) != len(mean):
                    for j in mean:
                        if j not in matches:
                            i = max(self.x.keys())+1
                            self.x[i] = x[j]
                            self.y[i] = y[j]
                            self.mean[i] = mean[j]
                            self.idd[i] = 0

            # initlize tracks
            if self.x == {}:
                self.x = x
                self.y = y
                self.mean = mean 
                for key in self.x:
                    self.idd[key] = 0

            # update tracks counter
            for j in self.x.keys():
                self.idd[j]+=1
                if self.idd[j] == 8:
                    self.idd.pop(j, None)
                    self.x.pop(j, None)
                    self.y.pop(j, None)
                    #self.xyz.pop(j,None)

            self.flag = 1

    def _xtion_rgb(self,imgmsg):
        self.xtion_img_rgb = self.cv_bridge.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough") 
        #self.xtion_img_rgb.setflags(write=True)                   # allow to change the values 
        self.xtion_img_rgb_original = self.xtion_img_rgb.copy()

        self._plot_results()    
        k = cv2.waitKey(1)
        
    def _plot_results(self):
        if self.flag != 0:
            # read the x,y, and mean location of objects
            self.x_use = self.x.copy()
            self.y_use = self.y.copy()
            self.mean_use = self.mean.copy()

            # initilize the images of the histograms
            for key in self.x_use:
                if key not in self.histograms:
                    self.histograms[key] = {}
                    self.histograms[key]['data'] = []
                    self.histograms[key]['counter'] = 0

            # remove deleted objects histograms
            remove = []
            for key in self.histograms:
                if key not in self.x_use:
                    remove.append(key)
            for key in remove:
                self.histograms.pop(key, None)
            self.flag = 0

            #print self.histograms
            histograms_img = np.zeros((160,160*len(self.histograms),3),dtype=np.uint8)
            # update the histograms
            for count,key in enumerate(self.x_use):
                if np.mod(self.frame_counter,1)==0:
                    # correct the orientation
                    img = self._calculate_orientation(key)
                    if img.shape[0] == 160 and img.shape[1] == 160:
                        # update the histograms
                        imgray = color.rgb2gray(img)
                        filtered = filters.sobel(imgray)  
                        filtered[filtered>0.1] = 1.0
                        N = self.histograms[key]['counter']
                        if N == 0:     self.histograms[key]['data'] = filtered
                        else:          self.histograms[key]['data'] += filtered
                        self.histograms[key]['counter'] += 1
                        #Hist,Img = hog(self.histograms[key]['data']/self.histograms[key]['counter'], orientations=8, pixels_per_cell=(16, 16), cells_per_block=(1, 1), visualise=True)
                        histograms_img[:,160*count:160*(count+1),:] = color.gray2rgb(self.histograms[key]['data']*255/self.histograms[key]['counter'])
            cv2.imshow('histograms',histograms_img)
            self.frame_counter += 1

    def _calculate_orientation(self,key):
        orient_img = np.zeros(self.xtion_img_rgb.shape, dtype=np.uint8)
        xc = self.mean_use[key][0]
        yc = self.mean_use[key][1]
        X = np.asarray(self.x_use[key])
        Y = np.asarray(self.y_use[key])
        orient_img[Y,X,:] = [255,255,255]
        orient_gray = cv2.cvtColor(orient_img,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(orient_gray,127,255,0)
        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(orient_img, contours, -1, (255,255,255), 10)
        label_img = label(orient_img[:,:,0])
        regions = regionprops(label_img)
        for count,props in enumerate(regions):
            #print 'count',count
            y0, x0 = props.centroid
            x0 = int(np.round(x0))
            y0 = int(np.round(y0))
            orientation = props.orientation
            break
        if orientation<0:   ang = -90-orientation*180/np.pi
        if orientation>=0:  ang = 90-orientation*180/np.pi

        img = transform.rotate(self.xtion_img_rgb_original.copy(),ang,resize=False, center=(xc,yc), order=1, mode='constant', cval=0, clip=True, preserve_range=False)
        img = img[yc-80:yc+80,xc-80:xc+80]
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







