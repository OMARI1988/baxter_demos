#! /usr/bin/env python2.7
# -*- coding: iso-8859-1 -*-

import roslib
import rospy
import PyKDL
from visualization_msgs.msg import Marker, MarkerArray
import visualization_msgs
from geometry_msgs.msg import Point
import numpy as np
from numpy import concatenate
import colorsys
from sklearn import mixture
import pickle
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def find_RGB_map(N):
	HSV_tuples = [(x*1.0/N, 1, 1) for x in range(N)]
	return map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples)

def find_window_size(N):
	a = 1
	b = 1
	flag = 0
	for i in range(N):
		if a*b >= N:
			break
		if flag == 0:
			a+=1
			flag=1
		else:
			b+=1
			flag=0
	return a,b

def read_file(x):

	directory = '/home/omari/ros_ws/src/baxter_demos/share/'
	pkl_file = open(directory+x+'.pkl', 'rb')
	print ' - loading data..'
	data1 = pickle.load(pkl_file)
	print ' - file loaded..'
	hyp = {}
	hyp['valid_HSV_hyp'] = []
	hyp['valid_dis_hyp'] = []
	hyp['valid_dir_hyp'] = []
	POINTS_HSV = data1['HSV']
	POINTS_SPA = data1['SPA']
	hyp = data1['hyp']
	return hyp

#used to paint the autovectors
def markerVector(id,vector,position):
    marker = Marker ()
    marker.header.frame_id = "/head_mount_kinect_link";
    marker.header.stamp = rospy.Time.now ()
    marker.ns = "my_namespace2";
    marker.id = id;
    marker.type = visualization_msgs.msg.Marker.ARROW
    marker.action = visualization_msgs.msg.Marker.ADD
    marker.scale.x=0.1
    marker.scale.y=0.3
    marker.scale.z=0.1
    marker.color.a= 1.0
    marker.color.r = 0.33*float(id)
    marker.color.g = 0.33*float(id)
    marker.color.b = 0.33*float(id)
    (start,end)=(Point(),Point())
 
    start.x = position[0]
    start.y = position[1]
    start.z = position[2]
    end.x=start.x+vector[0]
    end.y=start.y+vector[1]
    end.z=start.z+vector[2]
 
    marker.points.append(start)
    marker.points.append(end)
    #print str(marker)
    return marker
 
rospy.init_node ('markersample', anonymous = True)
points_pub = rospy.Publisher ("visualization_markers", visualization_msgs.msg.Marker,queue_size=1000)
gauss_pub = rospy.Publisher ("gaussian", visualization_msgs.msg.Marker,queue_size=1000)

hyp = read_file('16_colors')		# read a pickle file
	
word = 'red'				# test for this word

while not rospy.is_shutdown ():

    # build the data vector
    X = []
    x_val = hyp['hyp'][word]['hist_HSV_x']
    y_val = hyp['hyp'][word]['hist_HSV_y']
    z_val = hyp['hyp'][word]['hist_HSV_z']
    for i, (x, y, z) in enumerate(zip(x_val, y_val, z_val)):
	if i == 0:
		X = [[x/10.0,y/10.0,z/10.0]]
	else:
		X = np.vstack([X,[x/10.0,y/10.0,z/10.0]])

    # plot all data on rviz
    counter = 0
    for j, (x, y, z) in enumerate(zip(X[:,0], X[:,1], X[:,2])):
		#print j
		marker = Marker ()
		marker.header.frame_id = "/head_mount_kinect_link";
		marker.header.stamp = rospy.Time.now ()
		marker.ns = 'all'+str(j)
		marker.id = counter
		marker.type = visualization_msgs.msg.Marker.SPHERE
		marker.action = visualization_msgs.msg.Marker.ADD
		marker.pose.position.x = x
		marker.pose.position.y = y
		marker.pose.position.z = z
		marker.pose.orientation.x = 1
		marker.pose.orientation.y = 1
		marker.pose.orientation.z = 1
		marker.pose.orientation.w = 1
		marker.scale.x = 0.1
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.color.a = .7
		marker.color.r = 0
		marker.color.g = 0
		marker.color.b = 0
		points_pub.publish (marker)
		counter+=1
		rospy.sleep (.002)


    N = 5			# maximum number of clusters
    Y_all = []
    k_all = []
    clusters = []
    max_score_n = 0

    
    for cluster in range(1,N+1):
	    print 'number of clusters : ',cluster
	    clf = mixture.GMM(n_components=cluster, covariance_type='full')
	    clf.fit(X)

	    Y_ = clf.predict(X)

	    for k, (mean, covar) in enumerate(zip(clf.means_, clf._get_covars())):
		Np = float(len(X[Y_ == k,0]))
		Nm = float(hyp['hyp'][word]['counter_HSV'])
		#print Nm
		score_n = 1.0/(1.0 + np.abs(1.0 - Np/Nm))

		correct_pc = 0
		frame_counter = 0
		for frame in hyp['hyp'][word]['point_HSV_x']:
			frame_counter += 1
			x_val = hyp['hyp'][word]['point_HSV_x'][frame]
			y_val = hyp['hyp'][word]['point_HSV_y'][frame]
			z_val = hyp['hyp'][word]['point_HSV_z'][frame]
			for p in range(len(x_val)):
				if clf.predict([[x_val[p]/10.0,y_val[p]/10.0,z_val[p]/10.0]])[0] == k:
					correct_pc += 1
					break
		score_c = 1.0/(1.0 + np.abs(1.0 - float(correct_pc)/float(frame_counter)))
		print 'score_n for cluster ',k,' is : ',score_n
		print 'score_c for cluster ',k,' is : ',score_c

		if score_n*score_c > max_score_n:
			print 'we have a winner !'
			print score_n,score_c
			max_score_n = score_n*score_c
			#mean_n = mean
			#covar_n = covar
    			score_n_all = [score_n]
    			score_c_all = [score_c]
			Y_all = [Y_]
			k_all = [k]
			clusters = [cluster]
	



    RGB_tuples = find_RGB_map(len(Y_all))
    print 'clusters found = : ',len(Y_all)
    for k in range(len(Y_all)):
	Y_n = Y_all[k]
	k_n = k_all[k]
	cluster = clusters[k]
	score_n = score_n_all[k]
	score_c = score_c_all[k]
	print 'cluster ',k,' is from ',cluster,k_n
	print k,' cluster points = ',len(X[Y_n == k_n,0])
	print 'n score = ',score_n
	print 'n score = ',score_c

	for j, (x, y, z) in enumerate(zip(X[Y_n == k_n,0], X[Y_n == k_n,1], X[Y_n == k_n,2])):
		#print j
		marker = Marker ()
		marker.header.frame_id = "/head_mount_kinect_link";
		marker.header.stamp = rospy.Time.now ()
		marker.ns = 'cluster'+str(j)
		marker.id = j
		marker.type = visualization_msgs.msg.Marker.SPHERE
		marker.action = visualization_msgs.msg.Marker.ADD
		marker.pose.position.x = x
		marker.pose.position.y = y
		marker.pose.position.z = z
		marker.pose.orientation.x = 1
		marker.pose.orientation.y = 1
		marker.pose.orientation.z = 1
		marker.pose.orientation.w = 1
		marker.scale.x = 0.2
		marker.scale.y = 0.2
		marker.scale.z = 0.2
		marker.color.a = .7
		marker.color.r = RGB_tuples[k][0]
		marker.color.g = RGB_tuples[k][1]
		marker.color.b = RGB_tuples[k][2]
		#marker.color.r = 0
		#marker.color.g = 0
		#marker.color.b = 0
		points_pub.publish (marker)
		counter+=1
		rospy.sleep (.02)
		#print counter
    print '*************************************************************************************'
    rospy.spin()
 



    """
    #painting all the syntetic points
    for i in xrange (n_samples):
        #p = numpy.random.multivariate_normal (meanModel, covMatModel)
        p = X[i]
        if syntetic_samples == None:
            syntetic_samples =[p]
        else:
            syntetic_samples = concatenate ((syntetic_samples,[p]), axis = 0)
 	
        marker = Marker ()
        marker.header.frame_id = "/head_mount_kinect_link";
        marker.header.stamp = rospy.Time.now ()
        marker.ns = "my_namespace2";
        marker.id = i;
        marker.type = visualization_msgs.msg.Marker.SPHERE
        marker.action = visualization_msgs.msg.Marker.ADD
        marker.pose.position.x = p[0]
        marker.pose.position.y = p[1]
        marker.pose.position.z = p[2]
        marker.pose.orientation.x = 1
        marker.pose.orientation.y = 1
        marker.pose.orientation.z = 1
        marker.pose.orientation.w = 1
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        points_pub.publish (marker)
 
    #calculating Gaussian parameters
    print syntetic_samples
    syntetic_samples = np.array (syntetic_samples)
    covMat = np.cov (np.transpose (syntetic_samples))
    mean = np.mean ([syntetic_samples[: , 0], syntetic_samples[: , 1], syntetic_samples[:, 2]], axis = 1)
 
    #painting the gaussian ellipsoid marker
    marker = Marker ()
    marker.header.frame_id ="/head_mount_kinect_link";
    marker.header.stamp = rospy.Time.now ()
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs.msg.Marker.SPHERE
    marker.action = visualization_msgs.msg.Marker.ADD
    marker.pose.position.x = mean[0]
    marker.pose.position.y = mean[1]
    marker.pose.position.z = mean[2]
 
    #getting the distribution eigen vectors and values
    (eigValues,eigVectors) = np.linalg.eig (covMat)
 
    #painting the eigen vectors
    id=1
    for v in eigVectors:
        m=markerVector(id, v*eigValues[id-1], mean)
        id=id+1
        points_pub.publish(m)
 
    #building the rotation matrix
    eigx_n=PyKDL.Vector(eigVectors[0,0],eigVectors[0,1],eigVectors[0,2])
    eigy_n=-PyKDL.Vector(eigVectors[1,0],eigVectors[1,1],eigVectors[1,2])
    eigz_n=PyKDL.Vector(eigVectors[2,0],eigVectors[2,1],eigVectors[2,2])
    eigx_n.Normalize()
    eigy_n.Normalize()
    eigz_n.Normalize()
    rot = PyKDL.Rotation (eigx_n,eigy_n,eigz_n)
    quat = rot.GetQuaternion ()
 
    #painting the Gaussian Ellipsoid Marker
    marker.pose.orientation.x =quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]
    marker.scale.x = eigValues[0]*2
    marker.scale.y = eigValues[1]*2
    marker.scale.z =eigValues[2]*2
 
    marker.color.a = 0.5
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
 
    gauss_pub.publish (marker)
    rospy.sleep (.5)
    """
