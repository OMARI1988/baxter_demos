#! /usr/bin/env python2.7
# -*- coding: iso-8859-1 -*-

import rospy
import Tkinter
import numpy as np
from networkx import *
from baxter_demos.msg import obj_relations
from Learning_functions import *
from Plotting_functions import *

from interactive_markers.interactive_marker_server import *
from math import sqrt
from visualization_msgs.msg import *

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
#-----------------------------------------------------------------------------------------------------#
global f,obj_r,POINTS_HSV,POINTS_SPA,hyp
obj_r = []

POINTS_HSV = []		# the points used in plotting
POINTS_SPA = []		# the points used in plotting
hyp = {}		# the robot hypotheses
hyp['hyp'] = {}
hyp['valid_HSV_hyp'] = []
f = open('/home/omari/ros_ws/src/baxter_demos/share/language.txt', 'w')



#-----------------------------------------------------------------------------------------------------#
def Learning(sentence,o):
	global POINTS_HSV,POINTS_SPA,hyp

	plt.ion()

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

	o_color = {}
	o_color['H'] = o.H
	o_color['S'] = o.S
	o_color['V'] = o.V
	o_qsr = {}
	o_qsr['obj_number'] = len(o.id)
	count = 0
	for i in range(len(o.id)-1):
		for j in range(i+1,len(o.id)):
			o_qsr[str(i)+'-'+str(j)+'-dis'] = o.distance[count]
			o_qsr[str(i)+'-'+str(j)+'-ang'] = o.direction[count]
			count+=1
	print(' - Sentence: '+sentence)
	print(' - Parsing Sentence..')
	hyp = sentence_parsing(hyp,sentence)					# create a ser of words and increament the counter
	print(' - Update Histograms..')
	hyp,POINTS_HSV = Update_HSV_histogram(hyp,o_color,POINTS_HSV)		# update the histogram of HSV
	hyp = Test_HSV_Hypotheses(hyp,o_color,sentence)				# check to see if objects match any of the hypotheses

	if hyp['objects_hyp'][0] != []:
		POINTS_SPA = Update_SPA_histogram(o_qsr,POINTS_SPA)		# update the histogram of SPA

		#===========##============##===========$$============##===========##==============#	WITHOUT CK
		#hyp = Update_dis_histogram(hyp,o_qsr)		# update the histogram of distance WITHOUT CK
		#hyp = Update_dir_histogram(hyp,o_qsr)		# update the histogram of angle WITHOUT CK


		#===========##============##===========$$============##===========##==============#	WITH CK
		hyp = Update_dis_histogram_use_CK(hyp,o_qsr,sentence)		# update the histogram of distance
		hyp = Update_dir_histogram_use_CK(hyp,o_qsr,sentence)		# update the histogram of angle

	print(' - Compute Hypotheses..')
	hyp = Compute_HSV_hypotheses(hyp)		# Compute HSV hypotheses
	hyp = Compute_dis_hypotheses(hyp)		# Compute dis hypotheses 
	hyp = Compute_dir_hypotheses(hyp)		# Compute ang hypotheses

	print(' - Plotting Hypotheses..')
	Plotting_HSV_hypotheses(hyp,ax_HSV)
	Plotting(POINTS_HSV,ax_HSV,90,4,25,'HSV',0)

	#Plotting_SPA_hypotheses(hyp,ax_SPA)
	Plotting_dis_hypotheses(hyp,ax_DIS)
	Plotting(POINTS_SPA,ax_DIS,90,5,85,'Distance',0)

	Plotting_dir_hypotheses(hyp,ax_DIR)
	Plotting(POINTS_SPA,ax_DIR,90,6,85,'Direction',0)
	print(' - Done..')
	print(' ------------------')


	#print POINTS_HSV
        #int_marker = create_object_marker(idd,x,y,z)
        #server.insert(int_marker, objFeedback)
        #server.applyChanges()

#-----------------------------------------------------------------------------------------------------#
def make_msg(o):
	def add_msg(m,k):
	    for i in k:
		m+=str(i)+','
	    return m
		
	msg = 'id,'
	msg = add_msg(msg,o.id)
	msg+='id,H,'
	msg = add_msg(msg,o.H)
	msg+='H,S,'
	msg = add_msg(msg,o.S)
	msg+='S,V,'
	msg = add_msg(msg,o.V)
	msg+='V,dis,'
	msg = add_msg(msg,o.distance)
	msg+='dis,dir,'
	msg = add_msg(msg,o.direction)
	msg+='dir,'
	return msg

#-----------------------------------------------------------------------------------------------------#
def objects(data):
	global obj_r
	obj_r = data
	
#-----------------------------------------------------------------------------------------------------#
class simpleapp_tk(Tkinter.Tk):
    def __init__(self,parent):
        Tkinter.Tk.__init__(self,parent)
        self.parent = parent
        self.initialize()

    def initialize(self):
        self.grid()

        self.entryVariable = Tkinter.StringVar()
        self.entry = Tkinter.Entry(self,textvariable=self.entryVariable,width=130)
        self.entry.grid(column=0,row=0,sticky='EW')
        self.entry.bind("<Return>", self.OnPressEnter)
        self.entryVariable.set(u"Enter sentence.")

        button = Tkinter.Button(self,text=u"Add sentence",command=self.OnButtonClick)
        button.grid(column=1,row=0)

        button2 = Tkinter.Button(self,text=u"Finish",command=self.Finish)
        button2.grid(column=2,row=0)

        self.labelVariable = Tkinter.StringVar()
        label = Tkinter.Label(self,textvariable=self.labelVariable,
                              anchor="w",fg="white",bg="blue")
        label.grid(column=0,row=1,columnspan=4,sticky='EW')
        self.labelVariable.set(u"Ready !")

        self.grid_columnconfigure(0,weight=1)
        self.resizable(True,False)
        self.update()
        self.geometry(self.geometry())       
        self.entry.focus_set()
        self.entry.selection_range(0, Tkinter.END)

    def OnButtonClick(self):
	global obj_r
	msg = make_msg(obj_r)

	f.write(msg+self.entryVariable.get()+'\n')
	#print self.entryVariable.get()
	#print obj_r
        self.labelVariable.set( self.entryVariable.get()+" (Saved)" )
        self.entry.focus_set()
        self.entry.selection_range(0, Tkinter.END)
	Learning(self.entryVariable.get(),obj_r)

    def Finish(self):
	global f
	f.write('END')
	f.close()
	print 'Finish'
        self.labelVariable.set( "(Done)" )
        self.entry.focus_set()
        self.entry.selection_range(0, Tkinter.END)

    def OnPressEnter(self,event):
        self.labelVariable.set( self.entryVariable.get()+" (Saved)" )
        self.entry.focus_set()
        self.entry.selection_range(0, Tkinter.END)

#-----------------------------------------------------------------------------------------------------#
def initial():
	rospy.init_node('Language_GUI')
	rospy.Subscriber('/obj_relations', obj_relations, objects)
	server = InteractiveMarkerServer("object_features")


#-----------------------------------------------------------------------------------------------------#
if __name__ == "__main__":
    app = simpleapp_tk(None)
    app.title('Language Interface')
    rospy.loginfo('Language interface running..')
    initial()
    app.mainloop()

