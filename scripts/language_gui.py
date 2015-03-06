#! /usr/bin/env python2.7
# -*- coding: iso-8859-1 -*-

import rospy
import Tkinter
import numpy as np
from networkx import *
from baxter_demos.msg import obj_relations,obj_hypotheses
from Learning_functions import *
from Plotting_functions import *
import pickle
import inspect, os

from interactive_markers.interactive_marker_server import *
from math import sqrt
from visualization_msgs.msg import *

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
#-----------------------------------------------------------------------------------------------------#
global obj_r,Learn
obj_r = []

#-----------------------------------------------------------------------------------------------------#
class Learning:
    
    def __init__(self):
	self.directory = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
	self.directory = self.directory.replace("scripts", "share/");
	#print self.directory
	self.POINTS_HSV = []		# the points used in plotting
	self.POINTS_SPA = []		# the points used in plotting
	self.hyp = {}			# the robot hypotheses
	self.hyp['hyp'] = {}
	self.hyp['valid_HSV_hyp'] = []

	plt.ion()

	self.f_HSV=plt.figure(4)
	self.ax_HSV = self.f_HSV.add_subplot(111, projection='3d')
	self.f_HSV.suptitle('Robot Hypotheses in HSV', fontsize=20)
	self.ax_HSV.set_xlabel('X')
	self.ax_HSV.set_ylabel('Y')
	self.ax_HSV.set_zlabel('Z')

	self.f_DIS=plt.figure(5)
	self.ax_DIS = self.f_DIS.add_subplot(111, projection='3d')
	self.f_DIS.suptitle('Robot Hypotheses in Distance', fontsize=20)
	self.ax_DIS.set_xlabel('X')
	self.ax_DIS.set_ylabel('Y')
	self.ax_DIS.set_zlabel('Z')

	self.f_DIR=plt.figure(6)
	self.ax_DIR = self.f_DIR.add_subplot(111, projection='3d')
	self.f_DIR.suptitle('Robot Hypotheses in Direction', fontsize=20)
	self.ax_DIR.set_xlabel('X')
	self.ax_DIR.set_ylabel('Y')
	self.ax_DIR.set_zlabel('Z')
	

    def load(self,name):
	pkl_file = open(self.directory+name+'.pkl', 'rb')
	print ' - loading data..'
	data1 = pickle.load(pkl_file)
	print ' - file loaded..'
	self.POINTS_HSV = data1['HSV']
	self.POINTS_SPA = data1['SPA']
	self.hyp = data1['hyp']
	self.Plot()

    def save(self):
	data1 = {}
	data1['HSV'] = self.POINTS_HSV
	data1['SPA'] = self.POINTS_SPA
	data1['hyp'] = self.hyp
	output = open(self.directory+'data.pkl', 'wb')
	print ' - saving data..'
	pickle.dump(data1, output)
	print ' - data saved..'

    def language_and_vision(self,sentence,o):
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
	self.hyp = sentence_parsing(self.hyp,sentence)					# create a ser of words and increament the counter
	print(' - Update Histograms..')
	self.hyp,self.POINTS_HSV = Update_HSV_histogram(self.hyp,o_color,self.POINTS_HSV)		# update the histogram of HSV
	self.Test_hyp(o_color,[o.distance,o.direction])

	if self.hyp['objects_hyp'][0] != []:
		self.POINTS_SPA = Update_SPA_points(o_qsr,self.POINTS_SPA)		# update the histogram of SPA

		#===========##============##===========$$============##===========##==============#	WITHOUT CK
		#self.hyp = Update_dis_histogram(self.hyp,o_qsr)		# update the histogram of distance WITHOUT CK
		#self.hyp = Update_dir_histogram(self.hyp,o_qsr)		# update the histogram of angle WITHOUT CK


		#===========##============##===========$$============##===========##==============#	WITH CK
		self.hyp = Update_dis_histogram_use_CK(self.hyp,o_qsr,sentence)		# update the histogram of distance
		self.hyp = Update_dir_histogram_use_CK(self.hyp,o_qsr,sentence)		# update the histogram of angle

	print(' - Compute Hypotheses..')
	self.hyp = Compute_HSV_hypotheses(self.hyp)		# Compute HSV hypotheses
	self.hyp = Compute_dis_hypotheses(self.hyp)		# Compute dis hypotheses 
	self.hyp = Compute_dir_hypotheses(self.hyp)		# Compute ang hypotheses
	self.Plot()

    def Plot(self):
	print(' - Plotting Hypotheses..')
	Plotting_HSV_hypotheses(self.hyp,self.ax_HSV)
	Plotting(self.POINTS_HSV,self.ax_HSV,15,4,25,'HSV',0)

	#Plotting_SPA_hypotheses(self.hyp,ax_SPA)
	Plotting_dis_hypotheses(self.hyp,self.ax_DIS)
	Plotting(self.POINTS_SPA,self.ax_DIS,90,5,85,'Distance',0)

	Plotting_dir_hypotheses(self.hyp,self.ax_DIR)
	Plotting(self.POINTS_SPA,self.ax_DIR,90,6,85,'Direction',0)
	print(' - Done..')
	print(' ------------------')

    def Test_hyp(self,o_color,o_rel):
	self.hyp = Test_HSV_Hypotheses(self.hyp,o_color)				# check to see if objects match any of the hypotheses
	self.hyp = Test_Relation_Hypotheses(self.hyp,o_rel)				# check to see if objects match any of the hypotheses


        #int_marker = create_object_marker(idd,x,y,z)
        #server.insert(int_marker, objFeedback)
        #server.applyChanges()


#-----------------------------------------------------------------------------------------------------#
def objects(data):
	global obj_r,Learn,pub
	obj_r = data

	# for testing hypotheses
	o_color = {}
	o_color['H'] = obj_r.H
	o_color['S'] = obj_r.S
	o_color['V'] = obj_r.V
	Learn.Test_hyp(o_color,[obj_r .distance,obj_r .direction])
	if 'relations' in Learn.hyp:
		pub.publish(Learn.hyp['objects_hyp'],Learn.hyp['relations_hyp']['dis'],Learn.hyp['relations_hyp']['dir'],obj_r.Ximg,obj_r.Yimg)
	
	
#-----------------------------------------------------------------------------------------------------#
class simpleapp_tk(Tkinter.Tk):
    global obj_r,Learn
    def __init__(self,parent):
        Tkinter.Tk.__init__(self,parent)
        self.parent = parent
        self.initialize()
	Learn = Learning()
	self.directory = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
	self.directory = self.directory.replace("scripts", "share/");
	self.f = open(self.directory+'language.txt', 'w')

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

        button3 = Tkinter.Button(self,text=u"Save",command=self.Save)
        button3.grid(column=3,row=0)

        button4 = Tkinter.Button(self,text=u"Load",command=self.Load)
        button4.grid(column=4,row=0)

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
	msg = self.make_msg(obj_r)

	sentence = self.entryVariable.get()
	self.f.write(msg+sentence+'\n')
	#print sentence
	#print obj_r
        self.labelVariable.set( sentence+" (Saved)" )
        self.entry.focus_set()
        self.entry.selection_range(0, Tkinter.END)
	Learn.language_and_vision(sentence,obj_r)

    def Finish(self):
	#self.f.write('END')
	#self.f.close()
	print 'Finish'
        self.labelVariable.set( "Done" )
        self.entry.focus_set()
        self.entry.selection_range(0, Tkinter.END)

    def Save(self):
        #self.labelVariable.set( "Saving data..")
	Learn.save()
        self.labelVariable.set( "Data saved")
        self.entry.focus_set()
        self.entry.selection_range(0, Tkinter.END)

    def Load(self):
        #self.labelVariable.set( "Loading data..")
	file_name = self.entryVariable.get()
	if file_name == '':
		file_name = 'data'
	Learn.load(file_name)
        self.labelVariable.set( "Data Loaded")
        self.entry.focus_set()
        self.entry.selection_range(0, Tkinter.END)

    def OnPressEnter(self,event):
        self.labelVariable.set( self.entryVariable.get()+" (Saved)" )
        self.entry.focus_set()
        self.entry.selection_range(0, Tkinter.END)

    def make_msg(self,o):
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
def initial():
	global obj_r,Learn,pub
	rospy.init_node('Language_GUI')
	rospy.Subscriber('/obj_relations', obj_relations, objects)
	server = InteractiveMarkerServer("object_features")
	Learn = Learning()
	pub = rospy.Publisher('obj_hypotheses', obj_hypotheses, queue_size=1)


#-----------------------------------------------------------------------------------------------------#
if __name__ == "__main__":
    app = simpleapp_tk(None)
    app.title('Language Interface')
    rospy.loginfo('Language interface running..')
    initial()
    app.mainloop()

