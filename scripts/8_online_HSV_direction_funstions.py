#! /usr/bin/env python2.7
# -*- coding: iso-8859-1 -*-

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import cv2
import time
from Graph_functions import *
from Scene_functions import *
from QSR_functions import *
from Learning_functions import *
from Plotting_functions import *

Dir = '/home/omari/Desktop/Python/language/Simultaneous_learning_and_ground/images/'

#============================== Constants ======================================#


tests = 1			# for monte carlo test
n_examples = 2000			# number of examples in each test
plotting = 1			# choose to plot graphs or not
time = 1			# time to view each example
object_number_max = 4		# maximum number of objects allowed in one scene

display_frame = 5		# number of examples to update the robot brain
rotation_inc = 30		# rotation increament for the robot brain view



#-----------------------------------------------------------------------------------------------------#
if __name__ == "__main__":


	# Graph stuff
	plt.ion()
	f_world=plt.figure(1)
	f_world.suptitle('World features graph', fontsize=20)

	"""
	f2=plt.figure(2)
	plt.ion()
	plt.show()
	f2.suptitle('Robot hypothesis graph', fontsize=20)
	"""

	f_HSV=plt.figure(4)
	ax_HSV = f_HSV.add_subplot(111, projection='3d')
	f_HSV.suptitle('Robot Hypotheses in HSV', fontsize=20)
	ax_HSV.set_xlabel('X')
	ax_HSV.set_ylabel('Y')
	ax_HSV.set_zlabel('Z')

	f_DIS=plt.figure(5)
	ax_DIS = f_DIS.add_subplot(111, projection='3d')
	f_DIS.suptitle('Robot Hypotheses in Distance', fontsize=20)
	ax_DIS.set_xlabel('X')
	ax_DIS.set_ylabel('Y')
	ax_DIS.set_zlabel('Z')

	f_DIR=plt.figure(6)
	ax_DIR = f_DIR.add_subplot(111, projection='3d')
	f_DIR.suptitle('Robot Hypotheses in Direction', fontsize=20)
	ax_DIR.set_xlabel('X')
	ax_DIR.set_ylabel('Y')
	ax_DIR.set_zlabel('Z')

	POINTS_HSV = []		# the points used in plotting
	POINTS_SPA = []		# the points used in plotting
	hyp = {}		# the robot hypotheses
	hyp['hyp'] = {}
	hyp['valid_HSV_hyp'] = []

	for test in range(tests):

	#========================== MAIN LOOP ====================================#
		print '============================= Teaching ============================'
		for i in range(n_examples):
			o_loc,o_color,img = scene_gnerator(object_number_max)
			o_qsr = object_qsr(o_loc)
			sentence = scene_descreption(o_loc,o_color,o_qsr)
			print sentence
			print '-----------------------------------'

			cv2.imshow('Table',img)
			k = cv2.waitKey(1) & 0xFF

			# Learning start here:
			hyp = sentence_parsing(hyp,sentence)					# create a ser of words and increament the counter
			hyp,POINTS_HSV = Update_HSV_histogram(hyp,o_color,POINTS_HSV)		# update the histogram of HSV
			Update_world_graph(o_color,o_qsr,f_world)				# update the world features graph

			hyp = Test_HSV_Hypotheses(hyp,o_color,sentence)				# check to see if objects match any of the hypotheses

			if hyp['objects_hyp'][0] != []:
				#print '========================'
				hyp,POINTS_SPA = Update_SPA_histogram(hyp,o_qsr,POINTS_SPA)		# update the histogram of SPA

#===========##============##===========$$============##===========##==============#	WITHOUT CK
				#hyp = Update_dis_histogram(hyp,o_qsr)		# update the histogram of distance WITHOUT CK
				#hyp = Update_dir_histogram(hyp,o_qsr)		# update the histogram of angle WITHOUT CK


#===========##============##===========$$============##===========##==============#	WITH CK
				hyp = Update_dis_histogram_use_CK(hyp,o_qsr,sentence)		# update the histogram of distance
				hyp = Update_dir_histogram_use_CK(hyp,o_qsr,sentence)		# update the histogram of angle

				#print '========================'

			if (i+1)%display_frame == 0:
				hyp = Compute_HSV_hypotheses(hyp)		# Compute HSV hypotheses
				#hyp = Compute_SPA_hypotheses(hyp)		# Compute SPA hypotheses

				hyp = Compute_dis_hypotheses(hyp)		# Compute dis hypotheses 
				hyp = Compute_dir_hypotheses(hyp)		# Compute ang hypotheses

				Plotting_HSV_hypotheses(hyp,ax_HSV)
				#Plotting(POINTS_HSV,ax_HSV,5,4,25,'HSV',i)

				#Plotting_SPA_hypotheses(hyp,ax_SPA)
				Plotting_dis_hypotheses(hyp,ax_DIS)
				#Plotting(POINTS_SPA,ax_DIS,90,5,85,'Distance',i)

				Plotting_dir_hypotheses(hyp,ax_DIR)
				#Plotting(POINTS_SPA,ax_DIR,90,6,85,'Direction',i)

			cv2.imwrite(Dir+'Scene'+'-'+str(i+1)+'.png',img)
			cv2.imshow('Table',img)
			k = cv2.waitKey(time) & 0xFF










