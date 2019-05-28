#!/usr/bin/env python
import os
import os
import roslib
roslib.load_manifest('rtcus_kinect_gestures')
import rospy
import LbD
from LbD.GestureEntry import *
from LbD.DTWGestureModel import *
from LbD.GMMGestureModel import *
from LbD.HMMGestureModel import *
from GesturesDatabaseFacade import GestureDatabase

# number of Gaussian distributions (if optimal is not known beforehand, K must be set to None)
K = 5
# Maximum number of clusters possible, if optimal not known beforehand
MAX_CLUSTERS = 4
# threshold at which to stop the learning of GMM parameters
STOP_TH = 1e-9
    
def process_gesture_entry_simple_gesture(gesture_entry_name,gesture_db,negative_gestures_entries,models_types):
        rospy.loginfo("====== modeling gesture: %s ======",gesture_entry_name)
        
        #------------- GET CURRENT GESTURE -------------------
        rospy.loginfo("loading gesture: %s",gesture_entry_name)
        gesture=gesture_db.get_gesture_entry(gesture_entry_name)
        D=gesture.dimensionality
        
        if negative_gestures_entries!=None:
            rospy.loginfo("loading negative gestures: %s",negative_gestures_entries)
            negative_gestures_names= negative_gestures_entries.split(",")
            
            negative_gestures_entries=[]
            for neg_gest_name in negative_gestures_names:
                rospy.loginfo("loading negative gesture: %s",neg_gest_name)
                negative_gesture=gesture_db.get_gesture_entry(neg_gest_name)
                if negative_gesture.needs_accommodation():
                    negative_gesture.process_signal_accommodation(offset_accomodate=True,demo_longitude_accomodate=True,regular_sampling_acomodation=True)
                negative_gestures_entries.append(negative_gesture)
            assert len(negative_gestures_entries)>=1
            
        
        time_variable=gesture.process_signal_accommodation(offset_accomodate=True,demo_longitude_accomodate=True,regular_sampling_acomodation=True,time_wrapping=False).tolist()
        #FIRST TEST ALL MODELS WITHOUT TIME PRE-WRAPPING
        process_methods(gesture_entry_name,gesture_db,gesture,time_variable,D,negative_gestures_entries=negative_gestures_entries,models_types=models_types)
        
        #RELOAD GESTURE
        gesture=gesture_db.get_gesture_entry(gesture_entry_name)
        #PROCESS TIME-PRE-WRAPPING FOR MODELLING
        gesture.process_signal_accommodation(offset_accomodate=True,demo_longitude_accomodate=True,regular_sampling_acomodation=True,time_wrapping=True)
        time_variable=gesture.process_signal_accommodation(offset_accomodate=True,demo_longitude_accomodate=True,regular_sampling_acomodation=True).tolist()
        process_methods(gesture_entry_name,gesture_db,gesture,time_variable,D,file_postfix="time_wrapped",negative_gestures_entries=negative_gestures_entries,models_types=models_types)
      
def eval_and_print_likelihoods(gesture_entry,model):
    demo_training_set=gesture_entry.get_training_data(demo_index=0)
    rospy.loginfo("evaluating gesture_entry %s over a model; model.D=%d, gesture_entry.D=%d [gesture using time variable = %d]",gesture_entry.name,model.D,gesture_entry.dimensionality,gesture_entry.use_time_variable)
    results = []
        
    assert model.D==gesture_entry.dimensionality, "maybe you are using the time variable?"
    rospy.loginfo("-- evaluating demos --")
    for dem in xrange(gesture_entry.demonstration_count):
            demo_training_set=gesture_entry.get_training_data(demo_index=dem)
            demo_likelihood=model.compute_gesture_loglikelihood( demo_training_set)
            rospy.loginfo("demo %d\tlikelihood:%s   \t[%d samples]\t[%s]",dem,str(demo_likelihood),len(demo_training_set[0]),gesture_entry.demonstrations_names[dem])
            results.append({"demo_index":dem , "demo_name":gesture_entry.demonstrations_names[dem],"likelihood":demo_likelihood})
             
    return results

def end_store_and_evaluate_model(model_name,model,gesture_db,gesture_entry,negative_gestures_entries,use_time_variable=False):
        """
        @summary: finishes the process of creating an specific model
        """
        rospy.loginfo("auto positive samples likelihoods")
        eval_and_print_likelihoods(gesture_entry,model)
        
        if negative_gestures_entries!=None:
            #undo set time variable
            if use_time_variable:
                for g in negative_gestures_entries:
                    g.use_time_variable=True
            rospy.loginfo("negative samples likelihoods for computing threshold")
            for ng in negative_gestures_entries:
                eval_and_print_likelihoods(ng,model)
        
            threshold, precision,recall, positives_tests,total_tests =model.calculate_threshold([gesture_entry],negative_gestures_entries)
            rospy.loginfo("(threshold,precision,recall,positive_tests,total_tests)= (%f,%f,%f,%d,%d)",threshold,precision,recall,positives_tests,total_tests)
            #undo set time variable
            if use_time_variable:
                for g in negative_gestures_entries:
                    g.use_time_variable=False
        else:
            rospy.loginfo("no negative samples provided, threshold not computed")
            
        rospy.loginfo("model created... storing into the database")
        gesture_db.save_model(model_name, model)
                    
    
def process_methods(gesture_entry_name,gesture_db,gesture_entry,time_variable,D,file_postfix="",models_types="all",negative_gestures_entries=None):
        """
        @summary: creates a model of each type for an specific gesture
        """
        #------------ GMM MODEL WITH TIME VARIABLE ---------------
        if models_types=="all" or "Time-"+GMMGestureModel.__name__ in models_types:
            if K==None:
                gesture_entry.use_time_variable=True
                gmm_gesture_model=GMMGestureModel.create_and_learn(gesture_entry.name,D,gesture_entry.get_training_data(),STOP_TH,MAX_CLUSTERS)
                gesture_entry.use_time_variable=False
                        
            for k in xrange(1,K):
                rospy.loginfo("------------ Creating TIME GMM Model for %s",gesture_entry.name+" k="+str(k))
                
                sub_gesture_name=gesture_entry.name+".time.k%d%s"%(k,file_postfix)
                meta_tags=["GMM","clusters count: "+str(k),"time_variable"]
                if file_postfix!="":
                    meta_tags.append(file_postfix)
                gesture_entry.use_time_variable=True
                gesture_training_data=gesture_entry.get_training_data()
                
                gmm_gesture_model=GMMGestureModel(gesture_entry.name,gesture_entry.dimensionality,gesture_training_data , STOP_TH, k,meta_tags=meta_tags)
                gmm_gesture_model.learn()
                while len(gmm_gesture_model.get_gaussians()[0])!=k:
                    print "gaussian model with incorrect numnber of clusters: %d"%len(gmm_gesture_model.get_gaussians()[0])
                    gmm_gesture_model=GMMGestureModel(gesture_entry.name,gesture_entry.dimensionality,gesture_training_data , STOP_TH, k,meta_tags=meta_tags)
                    gmm_gesture_model.learn()
                    
                end_store_and_evaluate_model(sub_gesture_name,gmm_gesture_model,gesture_db,gesture_entry,negative_gestures_entries,use_time_variable=True)    
                gesture_entry.use_time_variable=False
        else:
            print "ignoring %s model. models_to_generate= %s"%("Time-"+GMMGestureModel.__name__,models_types)
            
        #-------------DTW MODEL --------------------------
        if models_types=="all" or DTWGestureModel.__name__ in models_types:
            rospy.loginfo("---------------- Creating DTW Model for %s",gesture_entry.name)
            meta_tags=["DTW"]
            if file_postfix!="":
                meta_tags.append(file_postfix)
                    
            dtwmodel=DTWGestureModel(gesture_entry,meta_tags=meta_tags)
            dtwmodel.learn()
            end_store_and_evaluate_model(gesture_entry.name+file_postfix,dtwmodel,gesture_db,gesture_entry,negative_gestures_entries)
        else:
            print "ignoring %s model. models_to_generate= %s"%("Time-"+DTWGestureModel.__name__,models_types)
        
                
        #------------GMM MODEL--------------
        if models_types=="all" or GMMGestureModel.__name__ in models_types:
            if K==None:
                gmm_gesture_model=GMMGestureModel.create_and_learn(gesture_entry.name,D,gesture_entry.get_merged_training_data(),STOP_TH,MAX_CLUSTERS)
                        
            for k in xrange(1,K):
                rospy.loginfo("-------------- Creating GMM Model for %s",gesture_entry.name+" k="+str(k))
                sub_gesture_name=gesture_entry.name+".k%d%s"%(k,file_postfix)
                training_data=gesture_entry.get_merged_training_data()
                meta_tags=["GMM","clusters count: "+str(k)]
                if file_postfix!="":
                    meta_tags.append(file_postfix)
                
                gmm_gesture_model=GMMGestureModel(gesture_entry.name,D, training_data, STOP_TH, k,meta_tags=meta_tags)
                gmm_gesture_model.learn()
                while len(gmm_gesture_model.get_gaussians()[0])!=k:
                    print "gaussian model with incorrect numnber of clusters: %d"%len(gmm_gesture_model.get_gaussians()[0])
                    gmm_gesture_model=GMMGestureModel(gesture_entry.name,D, training_data, STOP_TH, k,meta_tags=meta_tags)
                    gmm_gesture_model.learn()
                    
                end_store_and_evaluate_model(sub_gesture_name,gmm_gesture_model,gesture_db,gesture_entry,negative_gestures_entries)
        else:
            print "ignoring %s model. models_to_generate= %s"%(GMMGestureModel.__name__,models_types)
                   
        #------------HMM MODEL--------------- 
        if models_types=="all" or HMMGestureModel.__name__ in models_types:   
            for k in xrange(2,K):
                rospy.loginfo("--------------- Creating HMM Model for %s",gesture_entry.name+" k="+str(k))
                sub_gesture_name=gesture_entry.name+".k%d%s"%(k,file_postfix)
                meta_tags=["HMM","clusters count: "+str(k)]
                if file_postfix!="":
                    meta_tags.append(file_postfix)
    
                rospy.loginfo("loading GMM model "+ sub_gesture_name)
                gmm_gesture_model = gesture_db.get_model(sub_gesture_name, GMMGestureModel)
                
                hmm_gesture_model=HMMGestureModel(gmm_gesture_model,gesture_entry,STOP_TH,meta_tags=meta_tags)
                lgk=hmm_gesture_model.learn()
                
                while(numpy.isnan(lgk)):
                    rospy.loginfo("bad model, retrying")
                    hmm_gesture_model=HMMGestureModel(gmm_gesture_model,gesture_entry,STOP_TH,meta_tags=meta_tags)
                    lgk=hmm_gesture_model.learn()
                
                end_store_and_evaluate_model(sub_gesture_name,hmm_gesture_model,gesture_db,gesture_entry,negative_gestures_entries) 
        else:   
            print "ignoring %s model. models_to_generate= %s"%(HMMGestureModel.__name__,models_types)
                
    
        
if __name__ == '__main__':
    from optparse import OptionParser
    parser = OptionParser("%prog [ opts ]\nThis script processes sets of gesture bags to gesture.yaml files ")
    parser.add_option("-d","--directory", default=None,  dest="directory", help="directory where gesture database is located")
    parser.add_option("-g","--gesture-entry", default=None,  dest="gesture_entry", help="gesture entry name to be modeled")
    parser.add_option("-n","--negative-gestures-entries", default=None,dest="negative_gestures_entries",help="comma separated negative gestures names for the threshold calculation. Example: --negative-gestures-entries come_here_for_training,clapping")
    parser.add_option("-m","--models-types", default=None,dest="models_types",help="if no specified, all models are generated. Otherwise provide comma separated models names ie: -m HMMGestureModel,GMMGestureModel,Time-GMMGestureModel,DTWGestureModel")
    (options, args) = parser.parse_args()
    
    gesture_db=GestureDatabase()

    if options.directory!=None: 
        if os.path.exists(options.directory) and not os.path.isfile(options.directory):
            gesture_db.GESTURE_DATABASE_DIRECTORY=options.directory
        else:
            rospy.logfatal("Incorrect directory. exiting.")
            exit(-1)
    
    models_types="all"
    if options.models_types!= None:
        models_types=options.models_types.split(",")
        
    print "[%s] models to generate: %s"%(options.models_types,str(models_types))
    
    if options.gesture_entry!=None:
        #process just one specified gesture
        process_gesture_entry_simple_gesture(options.gesture_entry,gesture_db,options.negative_gestures_entries,models_types)
        exit()
    else:
        gesture_names=gesture_db.get_gesture_model_names()      
        for gesture_name in gesture_names:
            rospy.loginfo("current gestures: %s",str(gesture_names))
            process_gesture_entry_simple_gesture(options.gesture,gesture_db,options.negative_gestures_entries,models_types)        
