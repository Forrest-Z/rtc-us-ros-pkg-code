#!/usr/bin/env python
import numpy
from numpy import *

PKG = 'rtcus_kinect_gestures'
import roslib; 
roslib.load_manifest(PKG)
import rospy
import LbD
from LbD.GMMGestureModel import *
from LbD.GestureEntry import *
from GesturesDatabaseFacade import GestureDatabase

import sys
import unittest

# Maximum number of clusters possible, if optimal not known beforehand
MAX_CLUSTERS = 4
# threshold at which to stop the learning of GMM parameters
STOP_TH = 1e-6

## A sample python unit test
class TestGMMGestureModel(unittest.TestCase):
    
    def setUp(self):
        self.gesture_db=GestureDatabase()
        self.gesture= self.gesture_db.get_gesture_entry("come_here")
        self.D=self.gesture.dimensionality
        self.gesture.process_signal_accommodation()
        self.coordinates=self.gesture.get_merged_training_data()
        for i in xrange(len(self.coordinates)):
            for j in xrange(len(self.coordinates[i])):
                self.coordinates[i][j]= self.coordinates[i][j]*100.0
                
        self.__learning_system=None
        
    @property
    def  learning_system(self):
        if self.__learning_system==None:
            self.__learning_system = GMMGestureModel.create_and_learn(self.D,"test_gesture",self.coordinates,STOP_TH,MAX_CLUSTERS)
        return self.__learning_system
    
    def test_basic_comparation(self):
        a=self.learning_system
        return
    
    def test_check_auto_likelihood(self):
        other_gesture=copy(self.coordinates)
        other_gesture[0][0]=other_gesture[0][0]*2.0
        self.assertGreater(self.learning_system.get_training_set_likelihood(), self.learning_system.compute_gesture_likelihood(other_gesture))
        
    def test_save_load_model(self):
        filename=INPUT_DIR+"/models/temp_gmm_model.yaml"
        self.learning_system.save_to_file(filename)
        loaded= GMMGestureModel.load_from_file(filename)
        loaded_training= loaded.get_training_set_data()
        
        self.assertEquals(len(loaded_training),len(self.coordinates))
        self.assertEquals(len(loaded_training[0]),len(self.coordinates[0]))
        
        
if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'TestGMM', TestGMMGestureModel)