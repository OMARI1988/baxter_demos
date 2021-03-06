#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import roslib; roslib.load_manifest("interactive_markers")
import rospy

from interactive_markers.interactive_marker_server import *
from math import sqrt
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose,Point
from std_msgs.msg import ColorRGBA
import numpy as np
from itertools import combinations

global obj_id
obj_id = {}

rx = 0.0
ry = 0.7854
rz = 0.0
cy = np.cos(ry)
sy = np.sin(ry)

dx = 0.22
dy = 0.08
dz = 0.18
def camera_to_torso(p):

	# rotate frames around y axis and then offset them to match the kinect position on the robot torso
	P = Point()
	P.x = p.z 
	P.y = -(p.x+.042)
	P.z = -p.y 
	
	x = cy*P.x + sy*P.z
	z = -sy*P.x + cy*P.z
	
	P.x = x + dx
	P.y = P.y + dy
	P.z = z + dz
	return P
#--------------------------------------------------------------------------------------#
def objects(msg):
	global obj_id,server
	for i in msg.markers:
		data = len(i.points)
		
		P = camera_to_torso(i.points[0])
		x_min = P.x
		x_max = P.x
		y_min = P.y
		y_max = P.y
		z_min = P.z
		z_max = P.z

		for j in range(data):
			P = camera_to_torso(i.points[j])
			
			#k = i.points[j]
			if P.x < x_min:
				x_min = P.x
			if P.x > x_max:
				x_max = P.x
			if P.y < y_min:
				y_min = P.y
			if P.y > y_max:
				y_max = P.y
			if P.z < z_min:
				z_min = P.z
			if P.z > z_max:
				z_max = P.z

		


		if i.id not in obj_id:
			makeObjMarker(x_min,x_max,y_min,y_max,z_min,z_max,i.id)
			obj_id[i.id] = {}
			obj_id[i.id]['counter'] = 0
		else:
			server.erase('obj'+str(i.id))
			server.applyChanges()
			makeObjMarker(x_min,x_max,y_min,y_max,z_min,z_max,i.id)
			obj_id[i.id]['counter'] = 0

	for j in obj_id.keys():
		obj_id[j]['counter']+=1
		if obj_id[j]['counter'] == 5:
			obj_id.pop(j, None)
			server.erase('obj'+str(j))
			server.applyChanges()

def objFeedback( feedback ):
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        #compute difference vector for this cube
        x = feedback.pose.position.x
        y = feedback.pose.position.y
        z = feedback.pose.position.z

        server.applyChanges()

def create_object_marker( name, X, Y, Z ):
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/torso"
        int_marker.name = 'obj'+str(name)
        int_marker.description = 'obj'+str(name)
	
        pose = Pose()
        pose.position.x = .0
        pose.position.y = .0
        pose.position.z = .0
        int_marker.pose = pose

        line_marker = Marker()
        line_marker.type = Marker.LINE_LIST
        line_marker.scale.x = 0.005

        line_marker.points = []
        for i in range(len(X)):
                p = Point()
                p.x = X[i]
                p.y = Y[i]
                p.z = Z[i]
                line_marker.points.append(p)

        line_marker.colors = []
        for i in range(len(X)):
                color = ColorRGBA()
                color.r = .1
                color.g = .0
                color.b = .65
                color.a = .7
                line_marker.colors.append(color)

        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        control = InteractiveMarkerControl()
        control.markers.append(line_marker)
        int_marker.controls.append(control)

        return int_marker

def makeObjMarker(x_min,x_max,y_min,y_max,z_min,z_max,idd):

	x = []
	y = []
	z = []
	
	x.append(x_min);x.append(x_max);y.append(y_min);y.append(y_min);z.append(z_min);z.append(z_min);
	x.append(x_min);x.append(x_max);y.append(y_max);y.append(y_max);z.append(z_min);z.append(z_min);
	x.append(x_min);x.append(x_max);y.append(y_min);y.append(y_min);z.append(z_max);z.append(z_max);
	x.append(x_min);x.append(x_max);y.append(y_max);y.append(y_max);z.append(z_max);z.append(z_max);

	x.append(x_min);x.append(x_min);y.append(y_min);y.append(y_max);z.append(z_min);z.append(z_min);
	x.append(x_max);x.append(x_max);y.append(y_min);y.append(y_max);z.append(z_min);z.append(z_min);
	x.append(x_min);x.append(x_min);y.append(y_min);y.append(y_max);z.append(z_max);z.append(z_max);
	x.append(x_max);x.append(x_max);y.append(y_min);y.append(y_max);z.append(z_max);z.append(z_max);

	x.append(x_min);x.append(x_min);y.append(y_min);y.append(y_min);z.append(z_min);z.append(z_max);
	x.append(x_max);x.append(x_max);y.append(y_min);y.append(y_min);z.append(z_min);z.append(z_max);
	x.append(x_min);x.append(x_min);y.append(y_max);y.append(y_max);z.append(z_min);z.append(z_max);
	x.append(x_max);x.append(x_max);y.append(y_max);y.append(y_max);z.append(z_min);z.append(z_max);

        int_marker = create_object_marker(idd,x,y,z)
        server.insert(int_marker, objFeedback)
        server.applyChanges()


if __name__=="__main__":
    rospy.init_node("object_markers")
    
    server = InteractiveMarkerServer("object_markers")
    object_topic = rospy.resolve_name("/object_recognition_2/tabletop/clusters")
    rospy.Subscriber(object_topic, visualization_msgs.msg.MarkerArray, objects)

    rospy.loginfo("initializing object marker..")
    #server.applyChanges()

    rospy.spin()

