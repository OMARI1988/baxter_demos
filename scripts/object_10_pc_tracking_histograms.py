#! /usr/bin/env python2.7

import roslib
roslib.load_manifest('baxter_demos')
import rospy
import sensor_msgs.msg
import visualization_msgs.msg
from cv_bridge import CvBridge
import cv2
import numpy as np
from skimage import transform
import pickle
from std_msgs.msg import String

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
    
        self.test = 1
        self.Data = {}
        self.w = 80
        self.cv_bridge = CvBridge()	                # initilize opencv
        self.x = {}
        self.y = {}
        self.x1 = {}
        self.y1 = {}    
        self.mean1 = {}
        self.x_use = {}
        self.y_use = {}
        self.idd = {}
        self.xyz = {}
        self.histograms = {}
        self.fx = 525.0
        self.fy = 525.0
        self.cx = 319.5
        self.cy = 239.5
        self.flag = 0

        xtion_rgb_topic = rospy.resolve_name("/camera/rgb/image_color") 
        rospy.Subscriber(xtion_rgb_topic, sensor_msgs.msg.Image, self._xtion_rgb)

        object_topic = rospy.resolve_name("/object_recognition_2/tabletop/clusters")
        rospy.Subscriber(object_topic, visualization_msgs.msg.MarkerArray, self._clusters)
        
        sentence_topic = rospy.resolve_name("/sentence")
        rospy.Subscriber(sentence_topic, String, self._store_data)

        self.segment = scene_segment()
        
    def _clusters(self,cluster):  
        k = cv2.waitKey(1)
        # parsing Data
        x = {}
        y = {} 
        mean = {} 
        for key in cluster.markers:
            data = len(key.points)
            x[key.id] = np.zeros(data,dtype=int)
            y[key.id] = np.zeros(data,dtype=int)
            self.xyz[key.id] = {}
            self.xyz[key.id]['x'] = np.zeros(data,dtype=float)
            self.xyz[key.id]['y'] = np.zeros(data,dtype=float)
            self.xyz[key.id]['z'] = np.zeros(data,dtype=float)
            for j in range(data):
                k = key.points[j]
                self.xyz[key.id]['x'][j] = k.x
                self.xyz[key.id]['y'][j] = k.y
                self.xyz[key.id]['z'][j] = k.z
                #xyz[i.id]
                x[key.id][j] = int(k.x * self.fx / k.z + self.cx)
                y[key.id][j] = int(k.y * self.fy / k.z + self.cy)

            mean[key.id] = [int(np.mean(x[key.id])), int(np.mean(y[key.id]))]

        matches = []
        # update tracks
        if self.x != {}:
            # measure distance between means
            for i in self.mean:
                for j in mean:
                    dist = int(np.sqrt((self.mean[i][0] - mean[j][0])**2 + (self.mean[i][1] - mean[j][1])**2))
                    # do the update
                    if dist < 4:
                        matches.append(j)
                        self.x[i] = x[j]
                        self.y[i] = y[j]
                        self.mean[i] = mean[j]
                       

            # add new objects
            if len(matches) != len(mean):
                for j in mean:
                    if j not in matches:
                        i = max(self.x.keys())+1
                        self.x[i] = x[j]
                        self.y[i] = y[j]
                        self.mean[i] = mean[j]
                        self.idd[i] = 3

        # delete duplicated objects
        to_be_removed = []
        if self.x != {}:
            keys = self.mean.keys()
            for i in range(len(keys)-1):
                for j in range(i+1,len(keys)):
                    if self.mean[keys[i]] == self.mean[keys[j]]:
                        to_be_removed.append(j)
                        
            for j in to_be_removed:
                self.idd.pop(j, None)
                self.x.pop(j, None)
                self.y.pop(j, None)
                self.mean.pop(j, None)
                
        # initlize tracks
        if self.x == {}:
            self.x = x
            self.y = y
            self.mean = mean 
            for key in self.x:
                self.idd[key] = 3

        # update tracks counter
        for j in self.x.keys():
            if j in matches:
                self.idd[j]+=1
            else:
                self.idd[j]-=1
            if self.idd[j] < 0:
                self.idd.pop(j, None)
                self.x.pop(j, None)
                self.y.pop(j, None)
                self.mean.pop(j, None)
                #self.xyz.pop(j,None)
                
        keys = []
        for j in self.x.keys():
            if self.idd[j]>30:
                keys.append(j)
            if self.idd[j]>50:
                self.idd[j] = 50
        self.x1 = {}
        self.y1 = {}
        self.mean1 = {}
        for i in keys:
            self.x1[i] = self.x[i]
            self.y1[i] = self.y[i]
            self.mean1[i] = self.mean[i]

        if self.flag != 1:
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
            if self.x1 != {}:
                self.x_use = self.x1.copy()
                self.y_use = self.y1.copy()
                self.mean_use = self.mean1.copy()
                self.xyz_use = self.xyz.copy()

                # initilize the images of the histograms
                for key in self.x_use:
                    if key not in self.histograms:
                        self.histograms[key] = {}
                        self.histograms[key]['data'] = []
                        self.histograms[key]['img'] = np.zeros((self.w*2,self.w*2,3),dtype=float)
                        self.histograms[key]['counter'] = 0

                # remove deleted objects histograms
                remove = []
                for key in self.histograms:
                    if key not in self.x_use:
                        remove.append(key)
                for key in remove:
                    self.histograms.pop(key, None)

                #print self.histograms
                histograms_img = np.zeros((self.w*2,self.w*2*len(self.histograms),3),dtype=np.uint8)
                # update the histograms
                for count,key in enumerate(self.x_use):
                    # correct the orientation
                    img = self._find_histograms(key)
                    if img.shape[0] == self.w*2 and img.shape[1] == self.w*2:
                        histograms_img[:,self.w*2*count:self.w*2*(count+1),:] = img
                        cv2.putText( histograms_img,str(key), (self.w*2*count+5,140), cv2.FONT_HERSHEY_SIMPLEX, 1, 255)
                    #self._store_data(img,key)
                #self._compare_histograms()
                cv2.imshow('histograms',histograms_img)
            self.flag = 0

    def _compare_histograms(self):
        keys = self.histograms.keys()
        counter = 1
        for i in range(len(keys)-1):
            for j in range(i+1,len(keys)):
                c1 = self.histograms[keys[i]]['img']/self.histograms[keys[i]]['counter']
                c2 = self.histograms[keys[j]]['img']/self.histograms[keys[j]]['counter']
                c3 = cv2.flip(c2, 1)
                #cv2.imshow(str(counter),np.abs(c1-c3))
                #cv2.imshow('B',np.abs(c1-c2))
                cv2.waitKey(1)
                A = np.sum(np.abs(c1-c3))/10000
                B = np.sum(np.abs(c1-c2))/10000
                print keys[i],keys[j],' : ',np.min([A,B])
                counter += 1
                #print keys[i],keys[j],' : ',int(np.abs(np.sum(c1[:,:,0]-c3[:,:,0])))
        print '---------------'
        
    def _find_histograms(self,key):
        xc = self.mean_use[key][0]
        yc = self.mean_use[key][1]
        X = np.asarray(self.x_use[key])
        Y = np.asarray(self.y_use[key])
        img = np.zeros(self.xtion_img_rgb.shape, dtype=np.uint8)
        img[Y,X,:] = [255,255,255]
        #img[Y,xc-(X-xc),:] = [255,255,255]
        img = img[yc-self.w:yc+self.w,xc-self.w:xc+self.w]
        img = self._filter_image(img)
        img1 = img.astype(float)/255
        
        # basic correct of orientation
        ang = float(xc-320)/18
        img1 = transform.rotate(img1,ang,resize=False, center=(self.w,self.w), order=1, mode='constant', cval=0, clip=True, preserve_range=False)
        
        self.histograms[key]['img'] += img1.copy()
        self.histograms[key]['counter'] += 1 
        
        img2 = np.asarray((self.histograms[key]['img']/self.histograms[key]['counter']*255)).astype(np.uint8)
        return img2
        
    def _filter_image(self,img):
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,10,255,0)
        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, contours, -1, (255,255,255), 2)
        return img

    ############################################ remove key and image and store data from a callback based on the sentence recieved from the msg from language gui also incremeant the scene 
    def _store_data(self,msg):
        # store scene,shape,colour,xyz
        self.Data = {}
        print '--------------------------------------------'
        print 'saving scene number ',self.test,' with the following message :'
        self.Data['sentence'] = msg.data
        print self.Data['sentence']
        print 'saving scene ...'
        cv2.imwrite('/home/omari/Datasets/Lucas/'+str(self.test)+'_scene.png', self.xtion_img_rgb_original)

        print 'saving pickle file ...'
        for count,key in enumerate(self.x_use):
            img = self._find_histograms(key)
            img_color = np.zeros((40,40,3),dtype=np.uint8)
            X = np.asarray(self.x_use[key])
            Y = np.asarray(self.y_use[key])
            color = self.xtion_img_rgb_original[Y,X,:]
            B = int(np.mean(color[:,0]))
            G = int(np.mean(color[:,1]))
            R = int(np.mean(color[:,2]))
            img_color[:,:,:] = [B,G,R]
            cv2.imwrite('/home/omari/Datasets/Lucas/'+str(self.test)+'_shape_'+str(key)+'.png', img)
            cv2.imwrite('/home/omari/Datasets/Lucas/'+str(self.test)+'_color_'+str(key)+'.png', img_color)
            x1 = float(int(np.mean(self.xyz_use[key]['x'])*1000))/1000
            y1 = float(int(np.mean(self.xyz_use[key]['y'])*1000))/1000
            z1 = float(int(np.mean(self.xyz_use[key]['z'])*1000))/1000
            self.Data['scene'] = self.xtion_img_rgb_original
            self.Data[key] = {}
            self.Data[key]['color'] = [B,G,R]
            self.Data[key]['shape'] = self.histograms[key]['img']/self.histograms[key]['counter']
            self.Data[key]['x'] = x1
            self.Data[key]['y'] = y1
            self.Data[key]['z'] = z1
            self.Data[key]['xyz'] = self.xyz_use[key]
        pickle.dump(self.Data, open( '/home/omari/Datasets/Lucas/'+str(self.test)+'.p', "wb" ) )
        print 'Done saving.'
        print '--------------------------------------------'
        self.test += 1    
    
#--------------------------------------------------------------------------------------#
def main():
    object_detection()
    
    rospy.init_node('object_detection')
    rospy.loginfo('Object detection running..')
    rospy.spin()
    
#--------------------------------------------------------------------------------------#
if __name__ == '__main__':
    main()







