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
    
        self.test = 51
        self.Data = {}
        self.w = 80
        self.cv_bridge = CvBridge()	                # initilize opencv
        self.all_keys = [0]
        self.accepted_keys = {}
        self.accepted_keys_counter = 0
        self.x = {}
        self.y = {}
        self.mean = {}
        self.x1 = {}
        self.y1 = {}    
        self.mean1 = {}
        self.xyz1 = {}
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
        xyz = {}
        for key in cluster.markers:
            data = len(key.points)
            x[key.id] = np.zeros(data,dtype=int)
            y[key.id] = np.zeros(data,dtype=int)
            xyz[key.id] = {}
            xyz[key.id]['x'] = np.zeros(data,dtype=float)
            xyz[key.id]['y'] = np.zeros(data,dtype=float)
            xyz[key.id]['z'] = np.zeros(data,dtype=float)
            for j in range(data):
                k = key.points[j]
                xyz[key.id]['x'][j] = k.x
                xyz[key.id]['y'][j] = k.y
                xyz[key.id]['z'][j] = k.z
                #xyz[i.id]
                x[key.id][j] = int(k.x * self.fx / k.z + self.cx)
                y[key.id][j] = int(k.y * self.fy / k.z + self.cy)
            mean[key.id] = [int(np.mean(x[key.id])), int(np.mean(y[key.id]))]
            
        matches_new = []
        matches_old = []
        # update tracks
        # measure distance between means
        for i in self.mean:
            for j in mean:
                dist = int(np.sqrt((self.mean[i][0] - mean[j][0])**2 + (self.mean[i][1] - mean[j][1])**2))
                # do the update
                if dist < 4:
                    #print 'match found between existing',i,'and new',j
                    matches_new.append(j)
                    matches_old.append(i)
                    self.x[i] = x[j].copy()
                    self.y[i] = y[j].copy()
                    self.mean[i] = mean[j]
                    self.xyz[i] = xyz[j].copy() 
                    
        # add new objects
        if len(matches_new) != len(x):
            for j in x:
                if j not in matches_new:
                    #print 'new object found',j
                    i = max(self.all_keys)+1
                    self.all_keys.append(i)
                    self.x[i] = x[j].copy()
                    self.y[i] = y[j].copy()
                    self.mean[i] = mean[j]
                    self.xyz[i] = xyz[j].copy()
                    self.idd[i] = 3 
                    
        # update tracks counter
        for j in self.x.keys():
            if j in matches_old:    self.idd[j]+=1
            else:                   self.idd[j]-=1
            if self.idd[j] > 50:    self.idd[j]=50
            if self.idd[j] < 0:
                self.idd.pop(j, None)
                self.x.pop(j, None)
                self.y.pop(j, None)
                self.mean.pop(j, None)
                self.xyz.pop(j,None)
                
        # accept the good tracks  
        if self.flag != 1:              
            keys = []
            for j in self.x.keys():
                if self.idd[j]>30:
                    keys.append(j)
            self.x1 = {}
            self.y1 = {}
            self.mean1 = {}
            self.xyz1 = {}
            for i in keys:
                self.x1[i] = self.x[i].copy()
                self.y1[i] = self.y[i].copy()
                self.mean1[i] = self.mean[i]
                self.xyz1[i] = self.xyz[i].copy()
                """
                if i in self.accepted_keys:
                    print i,self.accepted_keys[i]
                    self.x1[self.accepted_keys[i]] = self.x[i].copy()
                    self.y1[self.accepted_keys[i]] = self.y[i].copy()
                    self.mean1[self.accepted_keys[i]] = self.mean[i]
                    self.xyz1[self.accepted_keys[i]] = self.xyz[i].copy()
                else:
                    self.accepted_keys[i] = self.accepted_keys_counter
                    self.accepted_keys_counter+=1
                    self.x1[self.accepted_keys[i]] = self.x[i].copy()
                    self.y1[self.accepted_keys[i]] = self.y[i].copy()
                    self.mean1[self.accepted_keys[i]] = self.mean[i]
                    self.xyz1[self.accepted_keys[i]] = self.xyz[i].copy()
                """
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
                self.xyz_use = self.xyz1.copy()
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

                histograms_img = np.zeros((self.w*2,self.w*2*len(self.histograms),3),dtype=np.uint8)
                # update the histograms
                for count,key in enumerate(self.x_use):
                    # correct the orientation
                    img = self._find_histograms(key)
                    self.histograms[key]['img'] += img.copy()
                    self.histograms[key]['counter'] += 1 
                    img_view = np.asarray((self.histograms[key]['img']/self.histograms[key]['counter']*255)).astype(np.uint8)
                    
                    if img.shape[0] == self.w*2 and img.shape[1] == self.w*2:
                        histograms_img[:,self.w*2*count:self.w*2*(count+1),:] = img_view
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
        # resize it
        height = np.max(Y)-np.min(Y)
        width = np.max(X)-np.min(X)
        window = int(np.max([width,height]))
        # make sure its withing the image size
        y1 = yc-window/2-5
        y2 = yc+window/2+5
        x1 = xc-window/2-5
        x2 = xc+window/2+5
        if y1<0 : y1=0
        if y2>480: y2=480
        if x1<0 : x1=0
        if x2>640: x2=640
        # crop the image to get the object
        img = img[y1:y2,x1:x2]
        img = self._filter_image(img)
        img = cv2.resize(img, (160, 160))
        img1 = img.astype(float)/255
        # basic correct of orientation
        ang = float(xc-320)/18
        img1 = transform.rotate(img1,ang,resize=False, center=(self.w,self.w), order=1, mode='constant', cval=0, clip=True, preserve_range=False)
        return img1
        
    def _filter_image(self,img):
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,10,255,0)
        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, contours, -1, (255,255,255), 2)
        return img
 
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
            img = np.asarray((self.histograms[key]['img']/self.histograms[key]['counter']*255)).astype(np.uint8)
            #img = self._find_histograms(key)
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

    def _correct_size(self,size_img):
        cnt = self._find_max_contour(size_img)
        fake_img = np.zeros((160,160,3),dtype=np.uint8)
        cv2.drawContours(fake_img, cnt, -1, (255,255,255), -1)
        cnt = cnt[0]
        leftmost = tuple(cnt[cnt[:,:,0].argmin()][0])[0]
        rightmost = tuple(cnt[cnt[:,:,0].argmax()][0])[0]
        topmost = tuple(cnt[cnt[:,:,1].argmin()][0])[1]
        bottommost = tuple(cnt[cnt[:,:,1].argmax()][0])[1]
        size = fake_img[leftmost:rightmost,topmost:bottommost,:].shape
        fake_img2 = np.zeros((size[0]+10, size[1]+10 ,3), dtype=np.uint8)
        fake_img2[5:5+size[0],5:5+size[1],:] = fake_img[leftmost:rightmost,topmost:bottommost,:]
        resized_image = cv2.resize(fake_img2, (160, 160)) 
        cv2.imshow('fake',resized_image)
        return resized_image

#--------------------------------------------------------------------------------------#
def main():
    object_detection()    
    rospy.init_node('object_detection')
    rospy.loginfo('Object detection running..')
    rospy.spin()
    
#--------------------------------------------------------------------------------------#
if __name__ == '__main__':
    main()







