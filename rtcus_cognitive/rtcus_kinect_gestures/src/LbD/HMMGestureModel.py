from stats_utils import *
import numpy
import sys
from scipy.cluster.vq import *
import LbD
from LbD.GestureModelBase import *
import mlpy
import ghmm

class HMMGestureModel(GestureModelBase):
    def __init__(self,gmm_gesture_model,gesture,STOP_TH,max_steps=5000,meta_tags=[]):
        GestureModelBase.__init__(self,gmm_gesture_model.gesture_name,gmm_gesture_model.D,meta_tags=meta_tags)
        
        self.stop_th=STOP_TH
        #observation probabilities
        self.gesture=gesture
        self.gmm_gesture_model=gmm_gesture_model
        
        (mus,sigmas,weights) =self.gmm_gesture_model.get_gaussians()
        self.K=len(mus)
        
        self.initial_probabilities= [1.0/self.K for _ in xrange(self.K)]
        self.transition_probabilities = [[1.0/self.K for _ in xrange(self.K)] for _ in xrange(self.K)]
        self.observation_probabilities=[]
        
        for i in xrange(self.K):
            observation_k=[mus[i].transpose().tolist()[0], self.__numpy_matrix_to_list(sigmas[i])]
            self.observation_probabilities.append(observation_k)
        
        self.max_steps=max_steps
    
    @property
    def F(self):
        return ghmm.Float()
    
    @property
    def markov_model(self):
        mm=ghmm.HMMFromMatrices(self.F,ghmm.MultivariateGaussianDistribution(self.F), self.transition_probabilities, self.observation_probabilities, self.initial_probabilities)
        #print ".>"+str(mm.asMatrices())
        return mm
    
    @property
    def training_sequences(self):
        return self.__gestures_markov_sequences_from_demos(self.gesture)
        
    #for gaussian's sigma matrix
    def __numpy_matrix_to_list(self,matrix):
        (rows,cols)=matrix.shape
        list_matrix=[]
        for i in xrange(rows):
            for j in xrange(cols):
                list_matrix.append(matrix[i,j])
        return list_matrix
        
    def comput_gesture_likelihood(self,gesture_data):
        return numpy.exp(self.compute_gesture_loglikelihood(gesture_data))
    
    def compute_gesture_loglikelihood(self,gesture_data):
        training_sequence=ghmm.EmissionSequence(self.F,self.__get_sequence_values_from_training_data(gesture_data))
        return self.markov_model.loglikelihood(training_sequence)
    
    def __get_sequence_values_from_training_data(self,training_data):
        values=[]
        samples=numpy.transpose(training_data)
        for sample in samples:
            for value in sample:
                values.append(value)
                
        #print "samples->"+str(samples)
        #print "values"+str(values)
        return values
        
    def __gestures_markov_sequences_from_demos(self,gesture):
        emission_sequences=[]
        for demo_index in xrange(gesture.demonstration_count):
            training_data=gesture.get_training_data(demo_index=demo_index)
            seq=self.__get_sequence_values_from_training_data(training_data)
            emission_sequences.append(seq)

        return ghmm.SequenceSet(self.F,emission_sequences)
         
    
    def stop_condition(self):
        return self._loglikelihood_current!=-1
        
    def learn_step(self):
        markov_model=self.markov_model
        training_sequences=self.training_sequences
        #print "created training sequence: "+ str(training_sequences)
        #likelihoods=markov_model.loglikelihoods(training_sequences)
        #print "before learn likelihoods ->" + str(likelihoods)
        
       
        #print "executing baulbWelch"
        markov_model.baumWelch(training_sequences,nrSteps=self.max_steps,loglikelihoodCutoff=self.stop_th)
        
        #print "after learn  likelihoods ->" + str(likelihoods)
        #print markov_model.asMatrices()

        (self.transition_probabilities,self.observation_probabilities,self.initial_probabilities)=markov_model.asMatrices()        
        likelihoods=markov_model.loglikelihoods(training_sequences)
        return numpy.mean(likelihoods)
         
  
        