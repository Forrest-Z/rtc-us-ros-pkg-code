#!/usr/bin/env python 
import os
import roslib
roslib.load_manifest('rtcus_kinect_gestures')
import rospy
import PyKDL
from geometry_msgs.msg import Point
from plot_rviz_utils import *
import os
import numpy
import random

import sys
import LbD
from LbD.GMMGestureModel import *
from LbD.GestureEntry import *
from LbD.stats_utils import *
from GesturesDatabaseFacade import GestureDatabase


# number of Gaussian distributions (if optimal is not known beforehand, K must be set to None)
K = 3
# Maximum number of clusters possible, if optimal not known beforehand
MAX_CLUSTERS = 5
# threshold at which to stop the learning of GMM parameters
STOP_TH = 1e-9
# Plot results (3D only)
PLOT = True

# MAIN
# -------------------------------------------------------

if __name__ == '__main__':
    from optparse import OptionParser
    parser = OptionParser("%prog [ opts ]\nThis examples take a gesture, creates the TMM model in the task space and show results in rviz")
    parser.add_option("-g","--gesture-entry", default="task_space_wave",  dest="gesture_entry", help="gesture to be get. Default: task_space_wave")
    (options, args) = parser.parse_args()
    
    rospy.init_node ('GMM_learning', anonymous = True)
    
    if PLOT:
    	clusters_pub = rospy.Publisher ("visualization_marker", Marker)
	gauss_pub = rospy.Publisher ("gaussian", Marker)
    
    gesture_db=GestureDatabase()
    rospy.loginfo("loading gesture entry: %s",options.gesture_entry)
    gesture=gesture_db.get_gesture_entry(options.gesture_entry)
    #print gesture.raw_data[0]
    gesture.process_signal_accommodation(offset_accomodate=True,demo_longitude_accomodate=True,regular_sampling_acomodation=True,time_wrapping=False)
    D=gesture.dimensionality
    coordinates=gesture.get_training_data(demo_index=0,scale=1.0)
    #explain the problem with singular matrixes ie: elbow -> shoulder... 2d
    
    
    rospy.loginfo("%s points loaded from database",str(len(coordinates[0])))
    coordinates= outlier_removal(coordinates)
    rospy.loginfo("outliers removed,  points len: %s",str(len(coordinates[0])))

    if K == None:
	   gmm_gesture_model = GMMGestureModel.create_and_learn(options.gesture_entry,D,coordinates,STOP_TH,MAX_CLUSTERS)
	   print "K opt = " + str(K)
    else:
	   gmm_gesture_model = GMMGestureModel(options.gesture_entry, D,coordinates,STOP_TH,K)
       
    K = len(gmm_gesture_model.get_gaussians()[2])
    print "K= %d"%K
       
    rospy.loginfo("learning system initializated")
    while not rospy.is_shutdown():

    	try:				
            #print "Likelihood: " + str(gmm_gesture_model.get_likelihood())
            # Plot datapoints in rviz
            #print ".. %d"% len(gmm_gesture_model.clusters_data_points[0])
            lastid=0
            (lastid,ms) = plot_points_set_rviz(gesture, gmm_gesture_model.clusters_data_points,startid=0)
            #print "its the time, markers: %d"%len(ms)
            for i,marker in enumerate(ms):
                #print "plotting marker %d"%i 
                clusters_pub.publish(marker)
    		
            (mus,sigmas,priors)=gmm_gesture_model.get_gaussians()
            # Plot gaussians in rviz
            (lastid,markers)=plot_gaussians_rviz(gesture,mus,sigmas,lastid,gauss_pub)
            #for marker in markers:
            #    gauss_pub.publish (marker)
            
            #if not gmm_gesture_model.stop_condition()
            #    gmm_gesture_model.learn_step()		
            rospy.sleep (0.1)
            rospy.loginfo("step gain %s",str(gmm_gesture_model.get_last_iteration_gain()))          
            
    	except KeyboardInterrupt:
    		print "Premature exit from loop by the user. Writing current results to output file"
    		break

# Results storage (XML)
