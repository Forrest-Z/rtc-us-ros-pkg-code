#!/usr/bin/env python 
import roslib
roslib.load_manifest('rtcus_kinect_gestures')
import rospy
import math
import os
import glob
import sys
import xml.dom.minidom
import tf
import datetime
import time
import numpy as np
from mpl_toolkits.mplot3d import axes3d, Axes3D
from visualization_msgs.msg import Marker, MarkerArray
import matplotlib.pyplot as plt
import PyKDL
import LbD
from LbD.GestureEntry import *
from plot_rviz_utils import *
import param_initialization

if __name__ == '__main__':
    rospy.init_node('register')
    
    rospy.loginfo("creating gesture")
    gesture_params=param_initialization.GestureParams()
    gesture=GestureEntry(gesture_params.GESTURE_NAME,gesture_params.SAMPLING_FREQUENCY,gesture_params.FRAMES,gesture_params.TIME_PER_DEMONSTRATION,gesture_params.NUM_DEMONSTRATIONS)  
    rospy.loginfo("gesture frames config: %s",[gesture.get_frame_config(i) for i in xrange(gesture.frame_count)])
    
    #ros variables
    if gesture_params.MARKERS_PLOT:
    	array_pub = rospy.Publisher('visualization_marker_array', Marker)
        # Creation of the current marker
        marker_count =0;
        markerArray= MarkerArray()
        marker = Marker()
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = MARKERS_SIZE*10
        marker.scale.y = MARKERS_SIZE*10
        marker.scale.z = MARKERS_SIZE*10
        marker.color.a = 1.0  
        
    listener = tf.TransformListener()
    rate = rospy.Rate(5)
    
    #main loop variables
    current_dem = 0   
    marker_count=0;
    while current_dem < gesture_params.NUM_DEMONSTRATIONS:
            rospy.loginfo("recording new gesture. Catching data for %s seconds.", gesture_params.TIME_PER_DEMONSTRATION)
            now=rospy.Time.to_sec(rospy.Time.now())
            end_time=now+gesture_params.TIME_PER_DEMONSTRATION
            rospy.loginfo("current time:%s, expected end time:%s",str(now),str(end_time))
            while rospy.Time.to_sec(rospy.Time.now())<end_time:
                for fr in xrange(gesture.frame_count):
                    (fixed_frame,target_frame,variables)=gesture.get_frame_config(fr)
                    rospy.logdebug("waiting frame.")
                    
                    listener.waitForTransform(fixed_frame, target_frame, rospy.Time(0), rospy.Time(120))
                    (trans,rot) = listener.lookupTransform( target_frame,fixed_frame, rospy.Time(0))
                    rospy.logdebug("frame catched.")
                    rot_rpy=PyKDL.Rotation.Quaternion(rot[0],rot[1],rot[2],rot[3]).GetRPY()
                    
                    gesture.push_frame_data_from_tf(fr,trans,rot_rpy,current_dem,rospy.Time.to_sec(rospy.Time.now()))
                    rospy.sleep(1.0/gesture_params.SAMPLING_FREQUENCY)
                    marker_count=marker_count+1;
                    
                    if MARKERS_PLOT:
                         (r,g,b)=getRGB_color_from_index(2*fr+current_dem,gesture_params.NUM_DEMONSTRATIONS*gesture.frame_count)
                         marker.color.r=r
                         marker.color.g=g
                         marker.color.b=b
                         marker.id = marker_count
                         marker.ns="ns_"+str(target_frame)
                         marker.header.frame_id = fixed_frame
                         marker.header.stamp=rospy.Time.now()
                         marker.pose.position.x = trans[0]
                         marker.pose.position.y = trans[1]
                         marker.pose.position.z = trans[2]
                         marker.pose.orientation.x = rot[0]
                         marker.pose.orientation.y = rot[1]
                         marker.pose.orientation.z = rot[2]
                         marker.pose.orientation.w = rot[3]
                         array_pub.publish(marker)
                    
            rospy.loginfo("Catching data Done. Samples: %d (sampling frecuency:%f)",marker_count, gesture_params.SAMPLING_FREQUENCY)
            rospy.loginfo("New demonstration register in " + str(gesture_params.WAIT_SECONDS) + " secs.")
            current_dem += 1
            rate.sleep()
    
    rospy.loginfo("saving gesture to file: %s",gesture_params.OUTPUT_DIR+gesture_params.OUTPUT_FILE_NAME)
    gesture.save_to_file(gesture_params.OUTPUT_DIR+gesture_params.OUTPUT_FILE_NAME)
    

