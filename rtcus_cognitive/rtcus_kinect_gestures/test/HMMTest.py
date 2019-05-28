#!/usr/bin/env python
import numpy
from numpy import *

PKG = 'rtcus_kinect_gestures'
import roslib; 
roslib.load_manifest(PKG)
import rospy
import LbD
from LbD.GMMGestureModel import *
from LbD.HMMGestureModel import *
from LbD.GestureEntry import *

import sys
import unittest

# name of the gesture to be learned
GESTURE_NAME = "wave"
# directory in which to look for the input files
INPUT_DIR = roslib.packages.get_pkg_dir('rtcus_kinect_gestures') + '/data'
# name of the file where the point cloud is stored
INPUT_FILE_NAME = INPUT_DIR+"/"+GESTURE_NAME+".yaml"

# Maximum number of clusters possible, if optimal not known beforehand
MAX_CLUSTERS = 4
# threshold at which to stop the learning of GMM parameters
STOP_TH = 1e-6

        
## A sample python unit test
class TestCustomHMMGestureModel(unittest.TestCase):
    
    def setUp(self):
        self.gesture=GestureEntry.load_from_file(INPUT_FILE_NAME)
        self.D=self.gesture.dimensionality
        self.coordinates=self.gesture.get_merged_training_data()
        self.learning_system = GMMGestureModel.create_and_learn(self.D,"test_gesture",self.coordinates,STOP_TH,MAX_CLUSTERS)
        
        for i in xrange(len(self.coordinates)):
            for j in xrange(len(self.coordinates[i])):
                self.coordinates[i][j]= self.coordinates[i][j]*100.0
                
    def test_instance(self):
        markov_system=CustomHMMGestureModel(selflearning_system)
        
    def test_instance(self):
        markov_system=CustomHMMGestureModel(self.learning_system)
        markov_system.learn()
        
    def test_ghmm(self):
           # this is being extended to also support mixtures of multivariate gaussians
            # Interpretation of B matrix for the multivariate gaussian case
            # (Example with three states and two mixture components with two dimensions):
            #  B = [
            #       [["mu111","mu112"],["sig1111","sig1112","sig1121","sig1122"],
            #        ["mu121","mu122"],["sig1211","sig1212","sig1221","sig1222"],
            #        ["w11","w12"] ],
            #       [["mu211","mu212"],["sig2111","sig2112","sig2121","sig2122"],
            #        ["mu221","mu222"],["sig2211","sig2212","sig2221","sig2222"],
            #        ["w21","w22"] ],
            #       [["mu311","mu312"],["sig3111","sig3112","sig3121","sig3122"],
            #        ["mu321","mu322"],["sig3211","sig3212","sig3221","sig3222"],
            #        ["w31","w32"] ],
            #      ]
            #
            # ["mu311","mu312"] is the mean vector of the two dimensional
            # gaussian in state 3, mixture component 1
            # ["sig1211","sig1212","sig1221","sig1222"] is the covariance
            # matrix of the two dimensional gaussian in state 1, mixture component 2
            # ["w21","w22"] are the weights of the mixture components
            # in state 2
            # For states with only one mixture component, a implicit weight
            # of 1.0 is assumed

            import ghmm            
            F = ghmm.Float()

            Abig = [[0.0,1.0],[1.0,0.0]]
            Bbig = [ [ [1.0,1.0,1.0],[0.9,0.4,0.2, 0.4,2.2,0.5, 0.2,0.5,1.0] ],
                          [ [10.0,10.0,10.0],[1.0,0.2,0.8, 0.2,2.0,0.6, 0.8,0.6,0.9] ] ]
            piBig = [0.5,0.5]
            modelBig = ghmm.HMMFromMatrices(F,ghmm.MultivariateGaussianDistribution(F), Abig, Bbig, piBig)
            modelBig.sample(10,100,seed=3586662)
            
            e=modelBig.sampleSingle(1)
            print [x for x in e]

            # get log P(seq | model)
            logp = model.loglikelihood(seq)
            print logp
              
            # cacluate viterbi path 
            path = model.viterbi(seq)
            print path
              
            # train model parameters
            model.baumWelch(seq_set,500,0.0001)
  

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'TestHMM', TestCustomHMMGestureModel)