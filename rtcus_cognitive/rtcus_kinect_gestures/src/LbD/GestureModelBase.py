import abc
import yaml
import numpy
from numpy import *
from abc import abstractmethod    

class GestureModelBase:
    
    #------------- properties --------------------
    @property
    def likelihood_previous(self):
        return self._loglikelihood_previous

    @property
    def likelihood_current(self):
        return self._loglikelihood_current
    
    @property
    def meta_tags(self):
        return self.__meta_tags
    
    @property
    def name(self):
        return self.__class__.__name__
    
    #TODO: REMOVE THESE METHODS (and refactoring all the references)
    def get_likelihood(self):
        if self.get_state()!= "learning":
            raise Exception("Invalid operation. The state must be 'learning'")
        else:
            return self._loglikelihood_current
    
    def get_training_set_likelihood(self):
        '''
        @return: get the likelihood of the training set points which made the GMM model
        '''
        if self.get_state()!= "learnt":
            raise Exception("Invalid operation. The state must be 'learnt'")
        else:
            return self._loglikelihood_current
    
    #------------ abstract methods ----------------    
    @abstractmethod
    def learn_step(self):
        '''must return the new model likelihood'''
        raise Exception("abstract method. Not Implemented")
    
    @abstractmethod
    def stop_condition(self):
        '''must return boolean'''
        raise Exception("abstract method. Not Implemented")
    
    @abstractmethod
    def compute_gesture_likelihood(self,gesture):
        '''must return the log-likelihood of the gesture'''
        raise Exception("abstract method. Not Implemented")
    
    def compute_gesture_loglikelihood(self,gesture):
        return numpy.log(self.compute_gesture_likelihood(gesture))
    
    @abstractmethod
    def get_training_set_data(self):
        raise Exception("abstract method. Not Implemented")
    #---------------------------------------------
        
    def __init__(self,gesture_name,dimensionality,meta_tags=[]):
        #dimensionality
        self.__gesture_name=gesture_name
        self.D=dimensionality
        self._loglikelihood_previous = -1.0
        self._loglikelihood_current=  -1.0
        self.__meta_tags=meta_tags
        self.__recognition_threshold=None
    
    @property
    def recognition_threshold(self):
        assert self.__recognition_threshold!=None, "recognition Threshold is not yet processed"
        return self.__recognition_threshold
    
    @property    
    def gesture_name(self):
        return self.__gesture_name
    
    @property
    def dimensionality(self):
        return self.D
    
    def learn(self):
        """@return: the aggregated loglikelihood of the training gestures. Tipically it is 
        an agreggation of the individual gestures likelihoods tough some models provides a single likelihood for a set of gestures  
        """
        while not self.stop_condition():
            new_likelihood=self.learn_step()
            self._loglikelihood_previous=self._loglikelihood_current
            self._loglikelihood_current=new_likelihood
        return self.likelihood_current
    
    
    def get_state(self):
        '''
        @return: string that specifies the current model state.
        Posibles values: unstarted, learnt, learning
        @see: get_states
        '''      
        if self._loglikelihood_previous==-1:
            return 'unstarted'
        else:
            if self.stop_condition():
                return "learnt"
            else:
                return "learning"
            
    def get_states(self):
        """
        @return: the possible states of the model. Posibles values: unstarted, learnt, learning
        @see: get_state
        """
        return ['unstarted','learning','learnt']
    
    def get_last_iteration_gain(self):
        """
        @return: the ratio betwen the current loglikelihood and the previous loglikelihood
        """
        return 1.0-(self._loglikelihood_current / self._loglikelihood_previous)
    
        
    def save_to_file(self,path):
        f=open(path, "w")
        yaml.dump(self,f)
        f.close()
        
    @staticmethod
    def load_from_file(path):
        f=open(path)
        gesture_model=yaml.load(f)
        return gesture_model
    
    def compute_likelihoods(self,gestures):
        results = []
        
        for gesture in gestures:        
            assert gesture.dimensionality==self.D
            
            for dem in xrange(gesture.demonstration_count):
                    demo_training_set=gesture.get_training_data(demo_index=dem)
                    demo_likelihood=self.compute_gesture_loglikelihood( demo_training_set)
                    results.append({"demo_index":dem , "gesture_name":gesture.name,"demo_name":gesture.demonstrations_names[dem],"likelihood":demo_likelihood})

        return results
                    
    def calculate_threshold(self,positive_gestures,negative_gestures):
        assert isinstance(positive_gestures,list)
        assert isinstance(negative_gestures,list)
        
        positive_training_list= self.compute_likelihoods(positive_gestures)
        negative_training_list= self.compute_likelihoods(negative_gestures)
        full_list =positive_training_list+negative_training_list
    
        posible_thresholds= sorted([ entry["likelihood"] for entry in full_list])
        
        quality_values = [ self.__calculate_clasificators_quality(positive_training_list,negative_training_list,threshold) for threshold in posible_thresholds ]
        best_index=numpy.argmax([ presition**2 + recall**2 for presition, recall, threshold in quality_values])
        
        self.__calculate_clasificators_quality(positive_training_list,negative_training_list,posible_thresholds[best_index])
        (presition,recall,threshold)= quality_values[best_index]
        self.__recognition_threshold=threshold
        
        return (threshold,presition,recall,len(positive_training_list),len(full_list))
        
    def __calculate_clasificators_quality(self,positive_training_list,negative_training_list,threshold):
        positivelikelihoods=[ entry["likelihood"] for entry in positive_training_list]
        negativelikelihoods=[ entry["likelihood"] for entry in negative_training_list]
    
        min_positive_likelihood = min([entry["likelihood"] for entry in positive_training_list])
        max_negative_likelihood = max([entry["likelihood"] for entry in negative_training_list])
    
        positive_success_list=[lk for lk in positivelikelihoods if lk>=threshold]
        positive_fails_list=[lk for lk in positivelikelihoods if lk<threshold]
        negative_success_list=[lk for lk in negativelikelihoods if lk<threshold]
        negative_fails_list=[lk for lk in negativelikelihoods if lk>=threshold]
        positive_success=len(positive_success_list)
        positive_fails=len(positive_fails_list)
        negative_success=len(negative_success_list)
        negative_fails = len(negative_fails_list)
    
        try:
            precision= double(positive_success)/double((positive_success+negative_fails))
            recall= double(positive_success)/double(len(positive_training_list))
            return (precision,recall,threshold)  
        except Exception,e:
            print e.message
            return (0.0, 0.0,threshold)
                     
        