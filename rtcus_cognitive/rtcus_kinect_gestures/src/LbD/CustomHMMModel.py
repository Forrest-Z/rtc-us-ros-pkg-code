from stats_utils import *
import numpy
import sys
from scipy.cluster.vq import *
import LbD
from LbD.GestureModelBase import *

class CustomHMMGestureModel(GestureModelBase):
    def __init__(self,gmm_gesture_model,gesture,meta_tags=[]):
        GestureModelBase.__init__(self,gmm_gesture_model.gesture_name,gmm_gesture_model.D,meta_tags)
        
        #observation probabilities
        self.gesture=gesture
        self.gmm_gesture_model=gmm_gesture_model
        self.gaussian_states =self.gmm_gesture_model.get_gaussians()
        self.K=len(self.gaussian_states[0])
        
        self.initial_probabilities = [0.0 for _ in xrange(self.K)]
        self.transition_probabilities = [[0.0 for _ in xrange(self.K)] for _ in xrange(self.K)]
        
        #get mu from the first gaussian
        self.D=len(self.gaussian_states[0][0]) 
        self.learnt=False
 
    def learn_step(self):
        return self.__basic_learn()
    
    def __basic_learn(self):
        
        for demo_index in xrange(self.gesture.demonstration_count):
            samples=self.gesture.get_training_data(demo_index=demo_index)
            sample_before=getPoint(samples,0)
            
            (mus,sigmas,_)= self.gaussian_states
            gaussians=zip(mus,sigmas)
     
            #first sample densities
            densities = [density(sample_before,self.D,mus[k],sigmas[k]) for k in xrange(self.K)]
            if self.initial_probabilities==None:
                self.initial_probabilities = numpy.array(densities)
            else:
                self.initial_probabilities = self.initial_probabilities+numpy.array(densities)
            
            before_state= numpy.argmax(densities)
            
            #counting transitions
            for s_index in xrange(1,len(samples[0])):
                sample=getPoint(samples,s_index)
                current_state = numpy.argmax([density(sample,self.D,mus[k],sigmas[k],True) for k in xrange(self.K)])
                #count transition
                self.transition_probabilities[before_state][current_state]+= 1.0
                before_state=current_state
        
        transition_normalization=[]
        #for each before state, get the total transitions
        for state_transitions in self.transition_probabilities:
            transition_normalization.append(sum(state_transitions))
        
        self.initial_probabilities=(self.initial_probabilities/numpy.linalg.norm(self.initial_probabilities)).tolist()
        #print "initial probs: "+ str(self.initial_probabilities)
        
        #print "transition table"
        #print self.transition_probabilities
        #print "transition table normalisation"
        for i in xrange(self.K):
            for j in xrange(self.K):
                if transition_normalization[i]>0.0:
                    self.transition_probabilities[i][j]=self.transition_probabilities[i][j]/transition_normalization[i]
                else:
                    self.transition_probabilities[i][j]= 1.0/numpy.double(self.K)
        
        #print self.transition_probabilities
        for demo_index in xrange(self.gesture.demonstration_count):
            samples=self.gesture.get_training_data(demo_index=demo_index)
            #print self.compute_gesture_likelihood(samples)
                
        self.learnt=True
        return self.compute_gesture_loglikelihood(samples)
    
    def stop_condition(self):
        return self.learnt
    
        
    def compute_gesture_likelihood(self,gesture_data):
        samples_count=len(gesture_data[0])
        
        obs_0= getPoint(gesture_data,0)
        (mus,sigmas,priors) = self.gaussian_states
        gaussians_count = len(mus)
        
        alpha_=[[-1.0 for _ in xrange(gaussians_count)] for _ in xrange(samples_count)]
        for k in xrange(gaussians_count):
            alpha_[0][k] = self.initial_probabilities[k] * density(obs_0,self.D,mus[k],sigmas[k]) 
            
        
        #print "alpha_0="+str(alpha_[0])
            
        for t in xrange(1,samples_count):
            obs_t= getPoint(gesture_data,t)
            for k in xrange(gaussians_count):
                alpha_[t][k] = sum([alpha_[t-1][k_bef]*self.transition_probabilities[k_bef][k] for k_bef in xrange(gaussians_count)]) * density(obs_t,self.D,mus[k],sigmas[k]) +1e-320
                #print "alpha_t=%s_k%s=%s" %(str(t),str(k),str(alpha_[t][k]))
        
        return sum([alpha_[-1][k] for k in xrange(gaussians_count)])/numpy.double(samples_count)
            