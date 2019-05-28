from stats_utils import *
import numpy
from numpy import double
import sys
from scipy.cluster.vq import *
import LbD
from LbD.GestureModelBase import *
import mlpy
from LbD.GestureEntry import GestureEntry
import dtw_utils

#[Senin08]    Pavel Senin. Dynamic Time Warping Algorithm Review

class DTWGestureModel(GestureModelBase):       
    def __init__(self,gesture,meta_tags=[]):
        assert isinstance(gesture,GestureEntry)
        GestureModelBase.__init__(self,gesture.name,gesture.dimensionality,meta_tags=meta_tags)
        self.gesture=gesture
        self.model_data=None
        
    def learn_step(self):
        distances=[0.0 for _ in xrange(self.gesture.demonstration_count)]
        
        for demo_a in xrange(self.gesture.demonstration_count):
            for demo_b in xrange(demo_a,self.gesture.demonstration_count):
                x=numpy.transpose(self.gesture.get_training_data(demo_index=demo_a))
                y=numpy.transpose(self.gesture.get_training_data(demo_index=demo_b))
                __constr=dtw_utils.dtw_no_window(len(x), len(y))
                (distance,distance_map)=dtw_utils.dtw_compute_asymmetric_with_map(x, y, __constr)
                distances[demo_a]+= distance**2
                distances[demo_b]+= distance**2
        
        #print "distances: "+str(distances)
        demo_model_index=numpy.argmin(distances)
        #print "....>>"+str(demo_model_index)
        self.model_data=self.gesture.get_training_data(demo_index=demo_model_index)
        
        return 0.0
    
    def stop_condition(self):
        return self.model_data!=None

    def compute_gesture_loglikelihood(self,gesture_data):
        time_sorted_gesture_data = numpy.transpose(self.model_data)
        time_sorted_model_data = numpy.transpose(gesture_data)
        
        __constr=dtw_utils.dtw_no_window(len(time_sorted_gesture_data),len(time_sorted_model_data))
        (dist,distance_map)=dtw_utils.dtw_compute_asymmetric_with_map(time_sorted_gesture_data,time_sorted_model_data,__constr)
        return - dist
    
    def compute_gesture_likelihood(self,gesture_data):
        return numpy.exp(self.compute_gesture_loglikelihood(gesture_data))    
    
    
 

    
    """def __mlpy_gesture_likelihood(self,gesture_data):
        dist_acc=0.0
        for var_index in xrange(len(gesture_data)):
            component_dist=self.dtw_processor.compute (self.model_data[var_index],gesture_data[var_index])
            dist_acc= dist_acc+component_dist**2.0
        return dist_acc**0.5
    """

        