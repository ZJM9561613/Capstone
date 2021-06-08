#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import sys
import rospy
import commands
import os,sys,string
import re
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String

def callback(data):
    id = data.markers[0].id
    distance = data.markers[0].pose.pose.position.z
    rospy.loginfo('I heard id=%d and distance=%f', id, distance)

    soundhandle = SoundClient()
    rospy.sleep(1)
    
    #denghuan
    stop_pub = rospy.Publisher('chatter', String, queue_size=1)
    rate = rospy.Rate(60)
    stop_msg = "run"
    if id < 20 and id > 0:
        rospy.loginfo('Playing sound %d.', id)
        for k in range(id):
            soundhandle.play(1, 1)
            rospy.sleep(0.5)
    
    if id == 0:
        if distance > 0.13:
            print("distance is larger than 13 centimeters")
        #denghuan
        else: 
            stop_msg = "stop"

    
    rospy.loginfo(stop_msg)
    stop_pub.publish(stop_msg)
    rospy.sleep(1)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.wait_for_message('ar_pose_marker', AlvarMarkers)

    rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
