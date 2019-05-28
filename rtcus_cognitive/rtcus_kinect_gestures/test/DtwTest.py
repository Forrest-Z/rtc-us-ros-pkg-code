#!/usr/bin/env python
import numpy
from numpy import *
import logging 

PKG = 'rtcus_kinect_gestures'
import roslib; 
roslib.load_manifest(PKG)
import rospy
import LbD
from LbD.DTWGestureModel import *

import sys
import unittest
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

## A sample python unit test
class TestDTW(unittest.TestCase):
    def setUp(self):
        logging.basicConfig( stream=sys.stdout )
        self.log=logging.getLogger( "TestDTW.output" )
        self.log.setLevel( logging.DEBUG )
    
    def test_basic(self):
        model_data= [[numpy.sin(x) for x in arange(100,step=0.1)]]
        dtw_model=DTWGestureModel(model_data)
        self.assertEquals(dtw_model.compute_gesture_likelihood(model_data),0.0)
        return
    
    def test_multiple(self):    
        model_data= [[numpy.sin(x) for x in arange(100,step=0.1)] for _ in xrange(4)]
        dtw_model=DTWGestureModel(model_data)
        self.assertEquals(dtw_model.compute_gesture_likelihood(model_data),0.0)
        return
    
    def test_multiple_2(self):    
        model_data= [[numpy.sin(x) for x in arange(100,step=0.1)] for i in arange(4.0)]
        test_data= [[i*numpy.sin(x) for x in arange(100,step=0.1)] for i in arange(4.0)]
        
        dtw_model=DTWGestureModel(model_data)
        self.assertGreater(dtw_model.compute_gesture_likelihood(test_data),0.0)
        return
    
    def test_DTW_comparison(self):
        import matplotlib
        sim_time=15.0
        time= arange(sim_time,step=0.05)
        model_data= [numpy.sin(t) for t in time]
        dtw_model=DTWGestureModel([model_data])
        
        test_data_1 = [numpy.sin(t*4.0) for t in time]
        test_data_2 = [4.0*numpy.sin(t) for t in time]
        test_data_3 = [numpy.cos(t) for t in time]
        
        self.log.debug("hi")               
        fig0 = matplotlib.pyplot.figure()
        matplotlib.pyplot.plot(time,model_data,"-")
        fig0.show()
        lk0=dtw_model.compute_gesture_likelihood([model_data])
        
        fig1 = matplotlib.pyplot.figure()
        matplotlib.pyplot.plot(time,test_data_1)
        fig1.show()
        lk1=dtw_model.compute_gesture_likelihood([test_data_1])
        
        fig2 = matplotlib.pyplot.figure()
        matplotlib.pyplot.plot(time,test_data_2)
        fig2.show()
        lk2=dtw_model.compute_gesture_likelihood([test_data_2])
        
        fig3 = matplotlib.pyplot.figure()
        matplotlib.pyplot.plot(time,test_data_3)
        fig3.show()
        lk3=dtw_model.compute_gesture_likelihood([test_data_3])
        
        test_data_4 = [numpy.sin(t) for t in arange(sim_time,step=0.025)]
        lk4=dtw_model.compute_gesture_likelihood([test_data_4])
        
        fig=matplotlib.pyplot.figure()
        
        
        amplitude = numpy.arange(2.0, 5, 0.1)
        frequency = numpy.arange(2.0, 5, 0.1)
        amplitude, frequency = numpy.meshgrid(amplitude, frequency)
        Z=numpy.zeros(shape=(len(amplitude),len(frequency)))
        
        for i in xrange(len(amplitude)):
            for j in xrange(len(frequency)):
                serie_a=[[amplitude[i][j]*numpy.sin(t) for t in time]]
                serie_b=[[numpy.sin(t*frequency[i][j]) for t in time]]
                
                Z[i][j] = numpy.exp(dtw_model.compute_gesture_likelihood(serie_a))/ numpy.exp(dtw_model.compute_gesture_likelihood(serie_b))
        
        fig_f=matplotlib.pyplot.figure()
        a=Axes3D(fig,azim=-200.0)
        
        for _ in xrange(4):
            a.azim=a.azim+60.0
            a.plot_surface(amplitude,frequency, Z)
            fig.show()
            sys.stdin.read(1)
            fig3.show()
        
        
        matplotlib.pyplot.plot(time,model_data,time,test_data_1,time,test_data_2)
        fig_f.show()
        sys.stdin.read(1)
        
if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'TestDTW', TestDTW)