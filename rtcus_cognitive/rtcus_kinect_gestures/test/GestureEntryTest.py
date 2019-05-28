#!/usr/bin/env python
import numpy
from numpy import *

PKG = 'rtcus_kinect_gestures'
import roslib; 
roslib.load_manifest(PKG)
import rospy
import LbD
from LbD.GestureEntry import *
import tf
import geometry_msgs.msg
import os

import sys
import unittest
# name of the gesture to be learned
GESTURE_NAME = "come_here"
# directory in which to look for the input files
INPUT_DIR = roslib.packages.get_pkg_dir('rtcus_kinect_gestures') + '/data/gestures/'
# name of the file where the point cloud is stored
INPUT_FILE_NAME = INPUT_DIR+"/"+GESTURE_NAME+".yaml"

from GesturesDatabaseFacade import BagDatabase

## A sample python unit test
class TestGestureEntryTest(unittest.TestCase):

    def tearDown(self):
        return
        
    def generate_gesture_aux(self):
        frames= []
        #frames: [ {fixed: /neck_1, target: /right_elbow_1,position: "xyz", orientation: "y"}, {fixed: /neck_1, target: /left_hand_1, position: "xyz", rotation: "y"}]
        sampling_period=0.1
        
        frames.append({'fixed':'/neck_1','target': '/right_elbow','position':'x','rotation':'y'})
        frames.append({'fixed':'/neck_1','target': '/left_elbow','position':'xy','rotation':'rpy'})
        
        gesture=GestureEntry('test gesture', frames,sampling_period**(-1))
        
        frame_index=0
        demo_index=0
        
        values=[arange(0.1,100.0,step=sampling_period),arange(0.1,100.0,step=sampling_period)]
        sample_index=0
        frame_time_offset=2.0
        frame_time_sampling_disalign=0.4*sampling_period
        
        count_a=0
        count_b=0                                  
        for t in arange(0.1,100.0,step=sampling_period):
            x=sin(t)
            yaw=sin(t)
            position = (x,0.0,0.0)
            rotation = (0.0,0.0,yaw)
            values[0][sample_index]=x
            values[1][sample_index]=yaw
            gesture.push_frame_data_from_tf(frame_index,position,rotation,demo_index,t)
            count_a=count_a+1              
            
            #the second frame a bit latter
            if(t>=frame_time_offset):
                gesture.push_frame_data_from_tf(frame_index+1,position,rotation,demo_index,t+frame_time_sampling_disalign)
                count_b=count_b+1
        
            sample_index=sample_index+1
        gesture.mark_gesture_defined()
        
        
        return (gesture,count_a,count_b,values)
    
    def generate_gesture(self):
        gesture,count_a,count_b,values= self.generate_gesture_aux()
        return GestureEntry.load_from_file(INPUT_FILE_NAME)
        return gesture
        
    def test_basic_inserted_coherence(self):
        gesture,count_a,count_b,values= self.generate_gesture_aux()
        self.assertEqual(gesture.get_temporal_info(0,0)["sampling_count"], count_a)
        self.assertEqual(gesture.get_temporal_info(1,0)["sampling_count"], count_b)
        gesture.use_time_variable=true
        coordinates=gesture.get_training_data(frame_index=0)
        self.assertEqual(len(coordinates[0]),count_a)
        coordinates=gesture.get_training_data(frame_index=1)
        self.assertEqual(len(coordinates[0]),count_b)
        
    def test_get_gesture_entry(self):
        gesture=self.generate_gesture()
        gesture.save_to_file("./temp.yaml")
        gesture=GestureEntry.load_from_file("./temp.yaml")
        
    def test_generate_gesture(self):
        self.generate_gesture()
        
    """
    def test_load_file(self):
        gesture=GestureEntry.load_from_file(INPUT_FILE_NAME)
        D=gesture.dimensionality
        coordinates=gesture.get_merged_training_data()
    """
    
    def test_calculate_time_info(self):
        gesture=self.generate_gesture()
        for demo_index in xrange(gesture.demonstration_count):
            for frame_index in xrange(gesture.frame_count):
                info=gesture.get_temporal_info(frame_index,demo_index)
                
    def test_a_signal_gesture_accomodation(self):
        gesture=self.generate_gesture()
        #gesture.process_signal_accommodation(offset_accomodate, demo_longitude_accomodate, sampling_count_accommodate)
        for demo_index in xrange(gesture.demonstration_count):
                self.assertTrue(any([gesture.get_temporal_info(frame_index,demo_index)["start_time"]!=0.0 for frame_index in xrange(gesture.frame_count)]))
        
        gesture.process_signal_accommodation(offset_accomodate=True, demo_longitude_accomodate=True)
        
        for demo_index in xrange(gesture.demonstration_count):
                self.assertTrue(any([gesture.get_temporal_info(frame_index,demo_index)["start_time"]==0.0 for frame_index in xrange(gesture.frame_count)]))
        
        demos_duration=[]
        for demo_index in xrange(gesture.demonstration_count):
            for frame_index in xrange(gesture.frame_count):
                info=gesture.get_temporal_info(frame_index,demo_index)["time_duration"]
                demos_duration.append(info)
        
        max_dist = max([max([abs(x-y) for x in demos_duration]) for y in demos_duration])
        sampling_period=  2.0*gesture.get_temporal_info(0,0)["sampling_period"]
        
        self.assertGreater(sampling_period,max_dist)
         

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'TestGestureEntry', TestGestureEntryTest)