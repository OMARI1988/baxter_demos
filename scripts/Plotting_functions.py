
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import operator
from itertools import product, combinations
import cv2
from matplotlib.patches import Circle, Wedge, Polygon
from matplotlib.collections import PatchCollection
import mpl_toolkits.mplot3d.art3d as art3d
import colorsys

Dir = '/home/omari/Python/language/Simultaneous_learning_and_ground/images2/'
#-------------------------------------------------------------------------------------#
def Plotting_dis_hypotheses(hyp,ax_SPA):

	
	#---------- find which words has hypotheses in HSV ----------#	
	plt.figure(5)
	plt.cla()

	N = len(hyp['valid_dis_hyp'])
	HSV_tuples = [(x*1.0/N, 1, 1) for x in range(N)]
	RGB_tuples = map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples)

	counter = 0
	for j in hyp['valid_dis_hyp']:

		X = hyp['hyp'][j]['dis_points_x_mean']
		Xs= hyp['hyp'][j]['dis_points_x_std']

	    	circle = Wedge((0,0), X-Xs, 0, 360, width=2*Xs,alpha=.5,color=RGB_tuples[counter])
	    	ax_SPA.add_patch(circle)
	  	art3d.pathpatch_2d_to_3d(circle, z=0.5, zdir='z')
		a = 0
		x = X*np.cos(a)
		y = X*np.sin(a)
		ax_SPA.text(1, 1-2*counter/float(N), .5, j,alpha=1,color=RGB_tuples[counter])
		counter += 1


#-------------------------------------------------------------------------------------#
def Plotting_dir_hypotheses(hyp,ax_SPA):

	
	#---------- find which words has hypotheses in HSV ----------#	
	plt.figure(6)
	plt.cla()

	N = len(hyp['valid_dir_hyp'])
	HSV_tuples = [(x*1.0/N, 1, 1) for x in range(N)]
	RGB_tuples = map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples)

	counter = 0
	for j in hyp['valid_dir_hyp']:

		X = hyp['hyp'][j]['dir_points_x_mean']
		Xs= hyp['hyp'][j]['dir_points_x_std']

	    	circle = Wedge((0,0), 1, X-Xs, X+Xs,alpha=.5,color=RGB_tuples[counter])

	    	ax_SPA.add_patch(circle)
	  	art3d.pathpatch_2d_to_3d(circle, z=0.5, zdir='z')
		a = 0
		x = X*np.cos(a)
		y = X*np.sin(a)
		ax_SPA.text(1, 1-2*counter/float(N), .5, j,alpha=1,color=RGB_tuples[counter])
		counter += 1

#-------------------------------------------------------------------------------------#
def Plotting_SPA_hypotheses(hyp,ax_SPA):

	
	#---------- find which words has hypotheses in HSV ----------#	
	plt.figure(5)
	plt.cla()

	for j in hyp['valid_SPA_hyp']:
		x1 = np.min(hyp['hyp'][j]['SPA_points_x'])
		#x2 = np.max(hyp['hyp'][j]['SPA_points_x'])
		y1 = np.min(hyp['hyp'][j]['SPA_points_y'])
		#y2 = np.max(hyp['hyp'][j]['SPA_points_y'])
		z1 = np.min(hyp['hyp'][j]['SPA_points_z'])
		#z2 = np.max(hyp['hyp'][j]['SPA_points_z'])

		X = hyp['hyp'][j]['SPA_points_x_mean']
		Y = hyp['hyp'][j]['SPA_points_y_mean']
		Z = hyp['hyp'][j]['SPA_points_z_mean']

		Xs= hyp['hyp'][j]['SPA_points_x_std']
		Ys= hyp['hyp'][j]['SPA_points_y_std']
		Zs= hyp['hyp'][j]['SPA_points_z_std']



		#draw sphere
		u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
		x=(np.cos(u)*np.sin(v))*Xs*2+X
		y=(np.sin(u)*np.sin(v))*Ys*2+Y
		z=(np.cos(v))*Zs*2+Z
		ax_SPA.plot_wireframe(x, y, z, color="gray",alpha=.5)
		x=(np.cos(u)*np.sin(v))*Xs*3+X
		y=(np.sin(u)*np.sin(v))*Ys*3+Y
		z=(np.cos(v))*Zs*3+Z
		ax_SPA.plot_wireframe(x, y, z, color="gray",alpha=.2)
		ax_SPA.text(x1, y1, z1, j, color="black",alpha=.7)

#-------------------------------------------------------------------------------------#
def Plotting_HSV_hypotheses(hyp,ax_HSV):

	
	#---------- find which words has hypotheses in HSV ----------#	
	plt.figure(4)
	plt.cla()

	for j in hyp['valid_HSV_hyp']:
		x1 = np.max(hyp['hyp'][j]['HSV_points_x'])
		#x2 = np.max(hyp['hyp'][j]['HSV_points_x'])
		y1 = np.max(hyp['hyp'][j]['HSV_points_y'])
		#y2 = np.max(hyp['hyp'][j]['HSV_points_y'])
		z1 = np.min(hyp['hyp'][j]['HSV_points_z'])
		#z2 = np.max(hyp['hyp'][j]['HSV_points_z'])

		X = hyp['hyp'][j]['HSV_points_x_mean']
		Y = hyp['hyp'][j]['HSV_points_y_mean']
		Z = hyp['hyp'][j]['HSV_points_z_mean']

		Xs= hyp['hyp'][j]['HSV_points_x_std']
		Ys= hyp['hyp'][j]['HSV_points_y_std']
		Zs= hyp['hyp'][j]['HSV_points_z_std']



		#draw sphere
		u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
		x=(np.cos(u)*np.sin(v))*Xs*2+X
		y=(np.sin(u)*np.sin(v))*Ys*2+Y
		z=(np.cos(v))*Zs*2+Z
		ax_HSV.plot_wireframe(x, y, z, color="gray",alpha=.5)
		x=(np.cos(u)*np.sin(v))*Xs*3+X
		y=(np.sin(u)*np.sin(v))*Ys*3+Y
		z=(np.cos(v))*Zs*3+Z
		ax_HSV.plot_wireframe(x, y, z, color="gray",alpha=.2)

		ax_HSV.text(x1, y1, z1, j, color="black",alpha=.7)

#-------------------------------------------------------------------------------------#
def Plotting(P,ax_HSV,rotation_inc,f,e,name,i):
		f = plt.figure(f)
		#---------- Plot HSV hypotheses ----------#
		ax_HSV.scatter([r[0] for r in P], [r[1] for r in P], [r[2] for r in P], c=[(r[3], r[4], r[5]) for r in P], marker='o')
		ax_HSV.set_xlim([-1,1])
		ax_HSV.set_ylim([-1,1])
		ax_HSV.set_zlim([0,1])

		counter = 0
		for kk in range(0,120,rotation_inc):
			ax_HSV.view_init(elev=e, azim=kk)
		   	plt.draw()
			#f.savefig(Dir+name+'-'+str(i+1)+'-'+str(counter)+'.png', dpi=400)
			counter+=1
			k = cv2.waitKey(1) & 0xFF


