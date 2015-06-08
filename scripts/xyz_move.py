#!/usr/bin/env python

#####################################################################################
#                                                                                   #
# Copyright (c) 2014, Active Robots Ltd.                                            #
# All rights reserved.                                                              #
#                                                                                   #
# Redistribution and use in source and binary forms, with or without                #
# modification, are permitted provided that the following conditions are met:       #
#                                                                                   #
# 1. Redistributions of source code must retain the above copyright notice,         #
#    this list of conditions and the following disclaimer.                          #
# 2. Redistributions in binary form must reproduce the above copyright              #
#    notice, this list of conditions and the following disclaimer in the            #
#    documentation and/or other materials provided with the distribution.           #
# 3. Neither the name of the Active Robots nor the names of its contributors        #
#    may be used to endorse or promote products derived from this software          #
#    without specific prior written permission.                                     #
#                                                                                   #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"       #
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE         #
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE        #
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE          #
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR               #
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF              #
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS          #
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN           #
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)           #
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE        #
# POSSIBILITY OF SUCH DAMAGE.                                                       #
#                                                                                   #
#####################################################################################

import rospy
import roslib

import cv;
import cv2;
import cv_bridge

import numpy
import math
import os
import sys
import string
import time
import random
import tf
from sensor_msgs.msg import Image
import baxter_interface
from moveit_commander import conversions
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import std_srvs.srv
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from baxter_demos.msg import obj_hypotheses, obj_location
# load the package manifest

# initialise ros node


#----------------------------------------------------------------------------#
# locate class
class manipulation():
    def __init__(self):
        self.right = "right"
        self.left = "left"
	self.fast = .5
	self.normal = .5
	self.slow = .2
	self.x_offset = .03
	self.y_offset = -.025
	self.z_offset = .05
        self.limb = ''

        self.right_limb_interface = baxter_interface.Limb(self.right)
        self.left_limb_interface = baxter_interface.Limb(self.left)

        # gripper ("left" or "right")
        self.right_gripper = baxter_interface.Gripper(self.right)
        self.left_gripper = baxter_interface.Gripper(self.left)

        # Enable the actuators
        baxter_interface.RobotEnable().enable()
	self.initial_position()

        # calibrate the gripper
        self.right_gripper.calibrate()
        self.left_gripper.calibrate()

    def initial_position(self):
        # set speed as a ratio of maximum speed
        self.right_limb_interface.set_joint_position_speed(self.fast)
        self.left_limb_interface.set_joint_position_speed(self.fast)

	# move right arm away
        self.baxter_ik_move(self.right, (0.3, -0.50, 0.2, math.pi, 0.0, 0.0))

	# move left arm away
        self.baxter_ik_move(self.left, (0.3, 0.50, 0.2, math.pi, 0.0, 0.0))


	# pick up an object
	#self.pick(x,y,z)

	# put down an object
	#self.put(x,y,z)

    #----------------------------------------------------------------------------#
    # move a limb
    def baxter_ik_move(self, limb, rpy_pose):
        quaternion_pose = conversions.list_to_pose_stamped(rpy_pose, "base")

        node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id="base")

        ik_request.pose_stamp.append(quaternion_pose)
        try:
            rospy.wait_for_service(node, 5.0)
            ik_response = ik_service(ik_request)
        except (rospy.ServiceException, rospy.ROSException), error_message:
            rospy.logerr("Service request failed: %r" % (error_message,))
            sys.exit("ERROR - baxter_ik_move - Failed to append pose")

        if ik_response.isValid[0]:
            print("PASS: Valid joint configuration found")
            # convert response to joint position control dictionary
            limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
            # move limb
            if 'right' == limb:
                self.right_limb_interface.move_to_joint_positions(limb_joints)
            else:
                self.left_limb_interface.move_to_joint_positions(limb_joints)
        else:
            # display invalid move message on head display
            #self.splash_screen("Invalid", "move")
            # little point in continuing so exit with error message
            print "requested move =", rpy_pose
            sys.exit("ERROR - baxter_ik_move - No valid joint configuration found")

    #----------------------------------------------------------------------------#
    def pick(self,x,y,z):

	self.initial_position()

	if y>0:
		self.gripper = self.left_gripper
		self.limb = self.left
		self.limb_interface = self.left_limb_interface
	else:
		self.gripper = self.right_gripper
		self.limb = self.right
		self.limb_interface = self.right_limb_interface

        self.gripper.open()

	x = x + self.x_offset
	y = y + self.y_offset
	z = z + self.z_offset

	# move above object
        self.baxter_ik_move(self.limb, (x, y, 0.2, math.pi, 0.0, 0.0))

        # slow down to reduce scattering of neighbouring
        self.limb_interface.set_joint_position_speed(self.slow)

	# pick up the object
        self.baxter_ik_move(self.limb, (x, y, z, math.pi, 0.0, 0.0))

        # close the gripper
        self.gripper.close()
        time.sleep(1)

	# left the object
        self.baxter_ik_move(self.limb, (x, y, 0.2, math.pi, 0.0, 0.0))

        # speed up again
        self.limb_interface.set_joint_position_speed(self.fast)
	if self.limb == 'left':
        	self.baxter_ik_move(self.limb, (0.3, 0.50, 0.2, math.pi, 0.0, 0.0))
	if self.limb == 'right':
        	self.baxter_ik_move(self.limb, (0.3, -0.50, 0.2, math.pi, 0.0, 0.0))


    #----------------------------------------------------------------------------#
    def put(self,x,y,z):

	if self.limb != '':	#pick should happen first

		x = x + self.x_offset
		y = y + self.y_offset
		z = z + self.z_offset

		# move above object
		self.baxter_ik_move(self.limb, (x, y, 0.2, math.pi, 0.0, 0.0))

		# slow down to reduce scattering of neighbouring
		self.limb_interface.set_joint_position_speed(self.slow)

		# pick up the object
		self.baxter_ik_move(self.limb, (x, y, z, math.pi, 0.0, 0.0))

		# close the gripper
        	self.gripper.open()
		time.sleep(1)

		# left the object
		self.baxter_ik_move(self.limb, (x, y, 0.2, math.pi, 0.0, 0.0))

		# speed up again
		self.limb_interface.set_joint_position_speed(self.fast)
		if self.limb == 'left':
			self.baxter_ik_move(self.limb, (0.3, 0.50, 0.2, math.pi, 0.0, 0.0))
		if self.limb == 'right':
			self.baxter_ik_move(self.limb, (0.3, -0.50, 0.2, math.pi, 0.0, 0.0))


#----------------------------------------------------------------------------#
class main():
    def __init__(self):
	rospy.init_node("manipulation", anonymous = True)
        rospy.Subscriber('/obj_hypotheses', obj_hypotheses, self.hypotheses)
        rospy.Subscriber('/obj_manipulation', obj_location, self.manipulation_callback)
        rospy.Subscriber('/obj_manipulation_voice', obj_hypotheses, self.manipulation_voice_callback)
        self.manipulation = manipulation()

    #----------------------------------------------------------------#
    def hypotheses(self,data):
	self.hyp = data
	
    #----------------------------------------------------------------#
    def manipulation_callback(self,data):
		x = data.X[0]-.01
		y = data.Y[0]+.02
		z = data.Z[0]+.04
		self.manipulation.pick(x,y,z)
		self.manipulation.put(x,y,z)

    #----------------------------------------------------------------#
    def manipulation_voice_callback(self,data):
	self.hyp2 = self.hyp
	for i in data.obj:
	    print i
	    if i in self.hyp2.obj:
		print '- ',i,' in hypotheses'
		ind = self.hyp2.obj.index(i)
		x = self.hyp2.X[ind]
		y = self.hyp2.Y[ind]
		z =  self.hyp2.Z[ind]
		self.manipulation.pick(x,y,z)
		self.manipulation.put(x,y,z)

if __name__ == "__main__":
    main()
    rospy.spin()

