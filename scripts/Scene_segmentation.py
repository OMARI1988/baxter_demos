#! /usr/bin/env python2.7

import roslib
roslib.load_manifest('baxter_demos')
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv2
import numpy as np
#import colorsys
from skimage import data, io, segmentation, color
from skimage.future import graph
from matplotlib import pyplot as plt, colors

if __name__ == '__main__':

    br = CvBridge()

    class scene_segment():
        def __init__(self):
            rospy.init_node('scene_segmentation')
            rospy.loginfo('scene segmentation running..')

            image_topic = rospy.resolve_name("/camera/rgb/image_color") 
            rospy.Subscriber(image_topic, sensor_msgs.msg.Image, self.read_image)
            self.img = []
            self.labels = []

        def _weight_mean_color(self,graph, src, dst, n):
            """Callback to handle merging nodes by recomputing mean color.

            The method expects that the mean color of `dst` is already computed.

            Parameters
            ----------
            graph : RAG
                The graph under consideration.
            src, dst : int
                The vertices in `graph` to be merged.
            n : int
                A neighbor of `src` or `dst` or both.

            Returns
            -------
            weight : float
                The absolute difference of the mean color between node `dst` and `n`.
            """

            diff = graph.node[dst]['mean color'] - graph.node[n]['mean color']
            diff = np.linalg.norm(diff)
            return diff


        def merge_mean_color(self,graph, src, dst):
            """Callback called before merging two nodes of a mean color distance graph.

            This method computes the mean color of `dst`.

            Parameters
            ----------
            graph : RAG
                The graph under consideration.
            src, dst : int
                The vertices in `graph` to be merged.
            """
            graph.node[dst]['total color'] += graph.node[src]['total color']
            graph.node[dst]['pixel count'] += graph.node[src]['pixel count']
            graph.node[dst]['mean color'] = (graph.node[dst]['total color'] /
                                             graph.node[dst]['pixel count'])


        def read_image(self,msg):
            self.img = br.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            cv2.imshow('original',self.img)

            if self.labels != []: 
         
                seg = color.label2rgb(self.labels, self.img, kind='avg')
                #print '5 label2rgb'
                seg = segmentation.mark_boundaries(seg, self.labels, (0, 0, 0))
                #print '6 mark_boundaries'
                cv2.imshow('quick segmentation',seg)
            k = cv2.waitKey(1) & 0xFF
            

        def set_parameters(self,labels):
            self.labels = labels



    S = scene_segment()

    while not rospy.is_shutdown():
        if S.img == []: continue
        print 'start'
        img = S.img.copy()
        labels = segmentation.slic(img, compactness=30, n_segments=200)
        S.set_parameters(labels)
        print labels
        print '1 segment'
        g = graph.rag_mean_color(img, labels)
        #print g.nodes()
        #print g.edges()
        print '2 rag_mean_color'
        out = graph.draw_rag(labels, g, img)

        #seg = color.label2rgb(labels, img, kind='avg')
        #print '5 label2rgb'
        #print '6 mark_boundaries'
        labels2 = {}
        #for i in range(90,100,20):
        i = 80


        seg = segmentation.mark_boundaries(img, labels, (0, 0, 0))
        cmap = colors.ListedColormap(['#6599FF', '#ff9900'])
        out1 = graph.draw_rag(labels, g, seg, node_color="#ffde00", colormap=cmap,
                     thresh=i, desaturate=True)

        labels2[i] = graph.merge_hierarchical(labels, g, thresh=i, rag_copy=False,
                                           in_place_merge=True,
                                           merge_func=S.merge_mean_color,
                                           weight_func=S._weight_mean_color)
        #S.set_parameters(labels2)
        #print '3 merging graphs'
        #g2 = graph.rag_mean_color(S.img, labels2)
        #print '4 rag mean colour'
        #for i in range(20,180,20):
        out2 = color.label2rgb(labels2[i], img, kind='avg')
        out2 = segmentation.mark_boundaries(out2, labels2[i], (0, 0, 0))
        cv2.imshow('final '+str(i),out2)

        cv2.imshow('graph',out)
        cv2.imshow('graph1',out1)
    rospy.spin()



