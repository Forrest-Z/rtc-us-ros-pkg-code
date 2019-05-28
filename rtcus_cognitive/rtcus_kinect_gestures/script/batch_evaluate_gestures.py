#!/usr/bin/env python
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
import batch_models_from_gesture_database
from batch_models_from_gesture_database import *
import yaml
import json
from paint_gesture_data import *
import datetime


REPORT_DIR=roslib.packages.get_pkg_dir('rtcus_kinect_gestures')+"/report" 

def create_table_from_report(report_data,report_meta_data):
    table = HTMLgen.Table( tabletitle='Classificators quality parameters', border=2, width=100, cell_align="right",heading=["","model_name","precision","recall","+ accepted (succ)","+ rejected (fail)","- rejected (succ)","- accepted (fail)","fitness(calculated)","optimum_threshold","target gesture likelihoods","others gestures likelihoods"])
    models_data= get_model_data(report_data)
    
    items= models_data.items()
    items.sort(key=lambda x: x[1]["fitness"],reverse=True)
    
    i=0
    for k,md in items:
        table.body.append([i,md["model_name"],md["precision"],md["recall"],md["postive success"],md["positive fails"],md["negative success"], md["negative fails"],md["fitness"],md["optimum_threshold"],md["positive_likelihoods"],md["negative_likelihoods"]])
        i+=1
    return table

def created_joined_table(entries,report_meta_data):
    table = HTMLgen.Table( tabletitle='Classificators quality parameters', border=2, width=100, cell_align="right",heading=["","model_name","precision","recall","+ accepted (succ)","+ rejected (fail)","- rejected (succ)","- accepted (fail)"])
    models_data= get_model_data(report_data)
    items= models_data.items()
    items.sort(key=lambda x: x[1]["fitness"],reverse=True)
    
    grouped_models={}
    for k,entry in items:
        model_entry_key= entry["model_name"]
        if not grouped_models.has_key(model_entry_key):
            grouped_models[model_entry_key]=[]
        
        grouped_models[model_entry_key].append(entry)
    
    i=0
    for k in grouped_models.keys():
        l=grouped_models[k]
        print "-----------"
        
        
        table.body.append([i,k,numpy.mean([a["precision"] for a in l]),numpy.mean([a["recall"] for a in l]),numpy.sum([a["postive success"] for a in l]),numpy.sum([a["positive fails"] for a in l]),numpy.sum([a["negative success"] for a in l]), numpy.sum([a["negative fails"] for a in l])])
        i+=1
    return table

        
def get_model_data(report_data):
    models_data={}
    
    #GROUP MODEL TESTS
    for entry_index,entry in enumerate(report_data):
        model_name=entry["model_name"]
        model_gesture_name=entry["model_gesture_name"]
        gesture_name=entry["gesture_name"]
        demo_index=entry["demo_index"]
        demo_name=entry["demo_name"]
        
        likelihood=entry["likelihood"]
        
        splitted_name=model_name.split(".")
        modeltype=splitted_name[-1]
        mname="".join(splitted_name[1:])
        
        #new data
        if not models_data.has_key(model_name):
            md={"model_name":mname,"model_gesture_name":model_gesture_name ,"positive_entries":[],"negative_entries":[],"recognition_threshold":entry["recognition_threshold"]}
            models_data[model_name]=md
        else:
            md=models_data[model_name]
        
        if  gesture_name==model_gesture_name:
            md["positive_entries"].append(entry_index)
        else:
            md["negative_entries"].append(entry_index)
             
    #ANALIZE MODEL TESTS      
    for k in models_data.keys():
        md=models_data[k]
        
        positive_entries=md["positive_entries"]
        negative_entries=md["negative_entries"]
        
        threshold=md["recognition_threshold"]
        print "recognition threshold: "+str(threshold)
        (presition,recall,threshold)=calculate_clasificators_quality(positive_entries,negative_entries,threshold,report_data,md,report=True)
        md["precision"]= presition
        md["recall"]= recall
        md["optimum_threshold"]= threshold
        md["fitness"]= numpy.sqrt(presition**2+recall**2)
        
    return models_data

def calculate_clasificators_quality(positive_training_list,negative_training_list,threshold,report_data,md,report=False):
    positivelikelihoods=[ report_data[entry]["likelihood"] for entry in positive_training_list]
    negativelikelihoods=[ report_data[entry]["likelihood"] for entry in negative_training_list]
    
    if report:
        md["positive_likelihoods"]=positivelikelihoods
        md["negative_likelihoods"]=negativelikelihoods
    
    """
    min_positive_likelihood = min([ report_data[entry]["likelihood"] for entry in positive_training_list])
    max_negative_likelihood = max([ report_data[entry]["likelihood"] for entry in negative_training_list])
    """
    
    positive_success_list=[lk for lk in positivelikelihoods if lk>=threshold]
    positive_fails_list=[lk for lk in positivelikelihoods if lk<threshold]
    negative_success_list=[lk for lk in negativelikelihoods if lk<threshold]
    negative_fails_list=[lk for lk in negativelikelihoods if lk>=threshold]
    positive_success=len(positive_success_list)
    positive_fails=len(positive_fails_list)
    negative_success=len(negative_success_list)
    negative_fails = len(negative_fails_list)
    
    if report:
        md["postive success"]= positive_success#+" "+str(positive_success_list)
        md["positive fails"]=positive_fails#+" "+str(positive_fails_list)
        md["negative success"]= negative_success#+" "+str(negative_success_list)
        md["negative fails"]=negative_fails#+" "+str(negative_fails_list)
    
    try:
        precision= double(positive_success)/double((positive_success+negative_fails))
        recall= double(positive_success)/double(len(positive_training_list))
        return (precision,recall,threshold)  
    except:
        print "fail calculation quality"  
        return (0.0, 0.0,threshold)


#================= MAIN ==========================================================
if __name__ == '__main__':
    
    #WARNING THIS CODE WILL NOT WORK IF THE PATH ENVIRONMENT DOES NOT INCLUDE THE RTCUS_KINECT_PACKAGE AND ITS NOT ONLY VALID THE SHELL
    from optparse import OptionParser
    parser = OptionParser("%prog [ opts ]\n")
    parser.add_option("-g","--gesture-entries", default=None,  dest="gesture_entries", help="gestures to be evaluated, ie:batch_evaluate_gestures.py --gestures come_here,wave,come_here_left_hand")
    parser.add_option("-r","--rebuild", default=False,  dest="rebuild", help="rebuild the data")
    parser.add_option("-o","--output",default="report_data", dest="output", help="output report name")
    parser.add_option("-m","--models-types", default=None,dest="models_types",help="models to evaluate. if no specified, all that are evaluated. Otherwise provide comma separated models names ie: -m HMMGestureModel,GMMGestureModel,Time-GMMGestureModel,DTWGestureModel")
    (options, args) = parser.parse_args()
    
    if options.gesture_entries!=None:
        gesture_entries_names=options.gesture_entries.split(",")
    else:
        #default gestures to evaluate
        rospy.loginfo("gestures must be specified using the parameter -g or (--gestures-entries). See help.")
        exit() 

    rospy.loginfo("gestures to be analyzed: %s",str(gesture_entries_names))    
    gesture_db=GestureDatabase()
    #gesture_entries_names=["come_here"]
    
    print "models to load: %s"%str(options.models_types)
    if options.models_types==None:
        models_names=gesture_db.get_models_names()
    else:
        models_types_name=options.models_types.split(",")
        models_names=[]
        for mt in models_types_name:
            models_names=models_names+gesture_db.get_models_names( model_type_name=mt)
            
    print "models found: %s"%str(models_names)
        
        
    #cache for avoiding load models
    models={}
    
    #gmm_gesture_model = gesture_db.get_model(model_gesture_name+".k1", GMMGestureModel)
    #hmm_gesture_model=gesture_db.get_model(model_gesture_name,HMMGestureModel)
    #dtwmodel=gesture_db.get_model(model_gesture_name,DTWGestureModel) 
    #models =[gmm_gesture_model,hmm_gesture_model,dtwmodel]
    
   
    
    import HTMLgen
    
    #---------- OPEN REPORT FILE ----------------
    report_file_name=REPORT_DIR+("/%s.json")%(options.output)
    rospy.loginfo("looking for existing file %s",report_file_name)
    
    if os.path.exists(report_file_name) and not options.rebuild:
        (report_data,report_meta_data)=json.load(open(report_file_name))
    else:
        report_data=[]
        report_meta_data={"models":{},"positive_list":[],"negative_list":[],"experiment_datetime":datetime.datetime.now().strftime("%a, %d %b %Y %H:%M:%S +0000")}
        #-------------------------EVALUATE MODELS-----------------
        for gesture_entry_name in gesture_entries_names:
            rospy.loginfo (" =========== gesture %s ===========",gesture_entry_name)
            rospy.loginfo("loading gesture...")
            gesture=gesture_db.get_gesture_entry(gesture_entry_name)
            rospy.loginfo("gesture loaded.")
            rospy.loginfo("gesture signal accomodation..")
            gesture.process_signal_accommodation(offset_accomodate=True,demo_longitude_accomodate=True,regular_sampling_acomodation=True)
            rospy.loginfo("gesture signal accomodation.Done")
                
            
            for model_name in models_names:
                rospy.loginfo("------ model: %s ------",model_name)
                rospy.loginfo("gesture to evaluate: %s ",gesture_entry_name)
            
                #Cache for maximize the speed of the report generation    
                if not models.has_key(model_name):
                    rospy.loginfo("loading model...")
                    model=gesture_db.get_model(model_name)
                    models[model_name]=model
                    rospy.loginfo("model loaded")
                else:
                    rospy.loginfo("getting cached model...")
                    model=models[model_name]
                    
               
                # ------end cache ---------
                rospy.loginfo("TAGS: %s || model_id:  %s",str(model.meta_tags),str(model))
                
                if not report_meta_data["models"].has_key( model_name):
                        report_meta_data["models"][model_name]=model.meta_tags
                
                if model.dimensionality==gesture.dimensionality+1:
                    gesture.use_time_variable=True
                else:
                    gesture.use_time_variable==False
                
                assert model.dimensionality==gesture.dimensionality
                #list of dictionaries with the results
                rospy.loginfo("Model threshold: %f",model.recognition_threshold)
                demo_likelihoods=eval_and_print_likelihoods(gesture,model)
                gesture.use_time_variable=False
                
                for entry in demo_likelihoods:
                    #append a field in the results
                    entry["model_name"]= model_name
                    entry["model_gesture_name"]=model.gesture_name
                    entry["gesture_name"]=gesture.name
                    entry["recognition_threshold"]=model.recognition_threshold
                    
                    demo_name=entry["demo_name"]
                    report_data.append(entry)
                    
                    keyname="model(%s)->"%(model.gesture_name)+demo_name
                    if  model.gesture_name== gesture.name:
                        if not keyname in report_meta_data["positive_list"]:
                            report_meta_data["positive_list"].append(keyname)
                    else:
                        if not keyname in report_meta_data["negative_list"]:
                            report_meta_data["negative_list"].append(keyname)
                    
            
        f=open(report_file_name, "w")
        json.dump((report_data,report_meta_data),f)
        f.close()
        
        print "report data:"
        print report_data
        print "report meta data:"
        print report_meta_data
        
        
    report=HTMLgen.SimpleDocument(title="Experimental validation of gestures recognition methods")

    #meta information of the experiment	
    report.append(HTMLgen.Heading(1,"Gesture recognition report"))
    report.append(HTMLgen.Div("Experiment datetime: %s"%report_meta_data["experiment_datetime"]))

    report.append(HTMLgen.Div("Total number of tests: %d"%len(report_data)))
    report.append(HTMLgen.Div("Total number of models: %d"%len(report_meta_data["models"])))

    report.append(HTMLgen.Heading(1,"Results"))
    report.append(HTMLgen.Div())

    #joined result table    
    table=created_joined_table(report_data, report_meta_data)
    table.width='100%'
    report.append(table)

    choices = ["Number of positive samples: "+str(len(report_meta_data["positive_list"])), report_meta_data["positive_list"], "Number of negative samples: "+str(len(report_meta_data["negative_list"])), report_meta_data["negative_list"]]
    report.append(HTMLgen.List(choices))
    
    gesture_models = ["come_here","wave"]
    for model_gesture_name in gesture_models:
        report.append(HTMLgen.Heading(2,"List of models of the gesture '%s'"% model_gesture_name))
        table=create_table_from_report([entry for entry in report_data if entry["model_gesture_name"]==model_gesture_name],report_meta_data)
        table.width='100%'
        report.append(table)
    
    print "generating images"
    plot_functions= get_plot_functions()
    generate_images(gesture_entries_names)
    for gesture_entry_name in gesture_entries_names:
    	for f in plot_functions:
            imageurl=REPORT_DIR+"/images/"+f.__name__+"/"+gesture_entry_name+".png"
            if os.path.exists(imageurl):
                rospy.loginfo("adding image to the report")
                div=HTMLgen.Div()
                div.append(HTMLgen.Heading(1,"gesture: "+str(imageurl)))
                div.append(HTMLgen.Image(imageurl))
                report.append(div)
            else:
                rospy.loginfo("image does not exist: %s"%imageurl)
    			
    report.write(REPORT_DIR+"/report.html")
