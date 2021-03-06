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
        #self.c = {}
        #self.xyz = {}
        self.fx = 525.0
        self.fy = 525.0
        self.cx = 319.5
        self.cy = 239.5
        self.idd = {}
        self.flag = 0
        self.main_counter = 0
        self.frame_counter = 0

        xtion_rgb_topic = rospy.resolve_name("/camera/rgb/image_color") 
        rospy.Subscriber(xtion_rgb_topic, sensor_msgs.msg.Image, self._xtion_rgb)

        object_topic = rospy.resolve_name("/object_recognition_2/tabletop/clusters")
        rospy.Subscriber(object_topic, visualization_msgs.msg.MarkerArray, self._clusters)
        
        self.segment = scene_segment()
        
    def _clusters(self,cluster):
        if self.flag != 1:
            for i in cluster.markers:
                data = len(i.points)
                self.x[i.id] = np.zeros(data,dtype=int)
                self.y[i.id] = np.zeros(data,dtype=int)
                #self.c[i.id] = [int(i.color.b*255), int(i.color.g*255), int(i.color.r*255)]
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
                    #self.c.pop(j, None)
                    #self.xyz.pop(j,None)

    def _xtion_rgb(self,imgmsg):
        self.xtion_img_rgb = self.cv_bridge.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough") 
        self.xtion_img_rgb.setflags(write=True)                   # allow to change the values 
        self.xtion_img_rgb_original = self.xtion_img_rgb.copy()

        self._plot_results()    
        cv2.imshow('xtion rgb',self.xtion_img_rgb_original)
        cv2.imshow('xtion rgb clustered',self.xtion_img_rgb)
        k = cv2.waitKey(1)
        
    def _plot_results(self):
        if self.flag != 0:
            self.x_use = self.x.copy()
            self.y_use = self.y.copy()
            self.flag = 0
        for key in self.x_use:
            x1 = min( self.x_use[key])
            x2 = max( self.x_use[key])
            y1 = min( self.y_use[key])
            y2 = max( self.y_use[key])
            cv2.rectangle(self.xtion_img_rgb, (x1,y1), (x2,y2), (255*np.random.rand(3)).astype(int), 2)
            cv2.imshow(str(key),self.xtion_img_rgb_original[y1-5:y2+5,x1-5:x2+5])
            if np.mod(self.frame_counter,10)==0:
                if self.main_counter < 10: msg = '0000'+str(self.main_counter)
                elif self.main_counter < 100: msg = '000'+str(self.main_counter)
                elif self.main_counter < 1000: msg = '00'+str(self.main_counter)
                elif self.main_counter < 10000: msg = '0'+str(self.main_counter)
                elif self.main_counter < 100000: msg = str(self.main_counter)
<<<<<<< HEAD:scripts/object_pc.py
                xc = (x1+x2)/2
                yc = (y1+y2)/2
                print xc,yc
                cv2.imwrite('/home/omari/Datasets/objects/coffee/coffee_'+msg+'.png',self.xtion_img_rgb_original[yc-80:yc+80,xc-80:xc+80])
                self.main_counter += 1
=======
                xc = np.mean(self.x_use)
                yc = np.mean(self.y_use)
                img = self.xtion_img_rgb_original[yc-80:yc+80,xc-80:xc+80]
                if img.shape[0] == 160 and img.shape[1] == 160:
                    cv2.imwrite('/home/omari/Datasets/objects/jug/jug_'+msg+'.png',img)
                    self.main_counter += 1
>>>>>>> 1f4a58de4b1f6d6bc09cdf605fb705ede6e5c5ab:scripts/object_5_pc.py
            self.frame_counter += 1
        #self.segment.segment_image(self.xtion_img_rgb[y1:y2,x1:x2],key)

            #self.xtion_img_rgb[self.y_use[key],self.x_use[key],:] = [255,0,0]
        """
        contour,hier = cv2.findContours(fgmask,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
        for count,cnt in enumerate(contour):
            area = cv2.contourArea(cnt)
            x,y,w,h = cv2.boundingRect(cnt)
            cx = x+w/2
            cy = y+h/2
            cw = int(w/2*self.enlarge)
            ch = int(h/2*self.enlarge)
            cv2.rectangle(img, (cx-cw,cy-ch), (cx+cw,cy+ch), (255*np.random.rand(3)).astype(int), 2)
            cv2.line(img,(cx-10,cy), (cx+10,cy), (255*np.random.rand(3)).astype(int), 2)
            cv2.line(img,(cx,cy-10), (cx,cy+10), (255*np.random.rand(3)).astype(int), 2)
            #roi=img[y:y+h,x:x+w]
            #cv2.imshow(str(count),roi)
        """
            



#--------------------------------------------------------------------------------------#
def main():
    object_detection()
    
    rospy.init_node('object_detection')
    rospy.loginfo('Object detection running..')
    rospy.spin()
    
#--------------------------------------------------------------------------------------#
if __name__ == '__main__':
    main()







