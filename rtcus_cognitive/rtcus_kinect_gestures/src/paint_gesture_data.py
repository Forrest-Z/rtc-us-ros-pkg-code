#!/usr/bin/env python
import roslib
roslib.load_manifest('rtcus_kinect_gestures')
import LbD.GestureEntry
import LbD

from LbD.GMMGestureModel import *
from LbD.GestureEntry import *
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.pyplot import * 
import numpy
from GesturesDatabaseFacade import GestureDatabase
import matplotlib.pyplot
from matplotlib.pyplot import *    
import matplotlib.patches
from plot_rviz_utils import gaussian_2d_ellipse_from_covariance_matrix

matplotlib.pyplot.rcParams['figure.figsize'] = [16, 16]
REPORT_DIR=roslib.packages.get_pkg_dir('rtcus_kinect_gestures')+"/report/images"


def print_gesture_info(gesture):
    # PRINT TEMPORAL SIGNAL INFORMATION
    gesture.use_time_variable=True
    for dem in xrange(gesture.demonstration_count):
        print "demo "+str(dem)
        for frame in xrange(gesture.frame_count):
            #fig1 = matplotlib.pyplot.figure()
            demo_samples=numpy.matrix(numpy.matrix(gesture.get_training_data(frame_index=frame,demo_index=dem)))
            #print str(gesture.get_temporal_info(frame,dem))
            #print demo_samples.shape
            
    
def plot_gesture_demonstrations(gesture,model,fig,max_number_demos,symbol="-",leg=[]):
    """
    @plot all demonstrations in different subplots. All trajectories are shown.
    """
    #PLOTTING SIGNALS
    plt.title(gesture.name)
    colours=["r","g","b","w","p"]
    print_gesture_info(gesture)
    
    #demo_indexes=xrange(min(gesture.demonstration_count,max_number_demos))
    demo_indexes=gesture.get_best_demos(max_number_demos)
    print max_number_demos
    gesture.use_time_variable=True
    for i,dem in enumerate(demo_indexes):
        plt.subplot(max_number_demos+1,1,i+1)
        plt.ylabel("joint space")
        
        if gesture.demonstrations_names!=None:
            plt.title("demo %d [%s] "%(dem,gesture.demonstrations_names[dem]))
        else:
            plt.title("demo %d"%(dem))
        
        for frame in xrange(gesture.frame_count):
            data=numpy.matrix(gesture.get_training_data(frame_index=frame,demo_index=dem)).transpose()
            (fixed_frame,target_frame,variables)=gesture.get_frame_config(frame)
            
            for (var_index,variable) in enumerate(variables):
                color =colours[var_index]
                plt.plot(data[:,0],data[:,var_index+1],symbol)
                leg.append("%s[%s] {ref:%s}"%(target_frame,variables[var_index],fixed_frame))
                
    return (gesture.name,leg)

def plot_separated_variables_gesture_demonstrations_with_dtw(gesture,model,fig,max_number_demos,symbol="-",leg=[]):
    gesture.process_signal_accommodation(offset_accomodate=True,demo_longitude_accomodate=True,regular_sampling_acomodation=True,time_wrapping=True)
    return plot_separated_variables_gesture_demonstrations(gesture,model,fig,max_number_demos,symbol,leg)
    

def plot_separated_variables_gesture_demonstrations(gesture,model,fig,max_number_demos,symbol="-",leg=[]):
    #PLOTTING SIGNALS
    plt.title(gesture.name)
    colours=["r","g","b","w","p"]
    
    gesture.use_time_variable=True
    plot_var_index=0
    #demo_indexes=xrange(min(gesture.demonstration_count,max_number_demos))
    demo_indexes=gesture.get_best_demos(max_number_demos)
    
    if model!=None:
        mus,sigmas,priors=model.get_gaussians()
        print "loading gaussians from model: mus=%s"%str(mus)
        
    for frame in xrange(gesture.frame_count):
        (fixed_frame,target_frame,variables)=gesture.get_frame_config(frame)
        for (frame_var_index,variable) in enumerate(variables):
            f=plt.subplot(gesture.dimensionality,1,plot_var_index+1)
            
            if model!=None:
                for k in xrange(len(mus)):
                    angle,scale,position=gaussian_2d_ellipse_from_covariance_matrix(sigmas[k], mus[k],[0,plot_var_index+1])
                    e=matplotlib.patches.Ellipse(position,scale[0],scale[1],angle)
                    e.set_alpha(0.1)
                    f.add_artist(e)
            
            for dem in demo_indexes:
                data=numpy.matrix(gesture.get_training_data(frame_index=frame,demo_index=dem)).transpose()
                
                #plt.ylabel("joint space")
                if gesture.demonstrations_names!=None:
                    plt.title("%s[%s] {ref:%s}"%(target_frame,variables[frame_var_index],fixed_frame))
                else:
                    plt.title("demonstration "+str(dem))
                
                color =colours[frame_var_index]
                plt.plot(data[:,0],data[:,frame_var_index+1],symbol)
            plot_var_index+=1
            leg.append("%s[%s] {ref:%s}"%(target_frame,variables[frame_var_index],fixed_frame))
                
    if model!=None:
        return (gesture.name+"-"+"".join(model.meta_tags),leg)
    else:
        return (gesture.name,leg)
    
def get_plot_functions():
    return [plot_gesture_demonstrations,plot_separated_variables_gesture_demonstrations,plot_separated_variables_gesture_demonstrations_with_dtw]

def generate_images(gestures_to_paint,time_wrapping=False):
    gesture_db=GestureDatabase()
    images=[]    
    max_number_demos=24
    functions=get_plot_functions()
    image_index=0
    
    for gesture_entry_name in gestures_to_paint:
        #load gesture
        gesture=gesture_db.get_gesture_entry(gesture_entry_name)
        print "loading gesture: %s"%gesture_entry_name
        
        try:
            
            #data preprocessing
            gesture.process_signal_accommodation(offset_accomodate=True,demo_longitude_accomodate=True,regular_sampling_acomodation=True,time_wrapping=False)
            
            for f in functions:
                #create plotting infrastructure
                fig1 = matplotlib.pyplot.figure(image_index)
                fig1.clear()
                leg=[]
                #plot
                number_of_demos=min(max_number_demos,gesture.demonstration_count)
                m=None
                (plot_key_name,leg)=f(gesture,m,fig1,number_of_demos)
                    
                if leg!=None:
                    legend(tuple(leg))
                    
                images.append((gesture.name,fig1))
                if m!=None:
                    imageurl=REPORT_DIR+"/"+f.__name__+"/"+gesture_entry_name+"-"+plot_key_name+".png"
                else:
                    imageurl=REPORT_DIR+"/"+f.__name__+"/"+gesture_entry_name+".png"
                fig1.savefig(imageurl)
                image_index+=1
        except:
            continue
            
        '''models=gesture_db.get_models_by_gesture_name(gesture.name+".time.*k2time_wrapped\.gmm")
        print "loading models: %s"+str(models)
        models= [None]+models
        
        for m in models:
            for f in functions:
                #create plotting infrastructure
                fig1 = matplotlib.pyplot.figure(image_index)
                fig1.clear()
                leg=[]
                #plot
                number_of_demos=min(max_number_demos,gesture.demonstration_count)
                (plot_key_name,leg)=f(gesture,m,fig1,number_of_demos)
                    
                if leg!=None:
                    legend(tuple(leg))
                    
                images.append((gesture.name,fig1))
                imageurl=REPORT_DIR+"/"+f.__name__+"/"+gesture_entry_name+"-"+plot_key_name+".png"
                fig1.savefig(imageurl)
                image_index+=1
        '''
        
        
    return images
        
#MAIN
if __name__ == "__main__":
    #gestures_to_paint=["come_here_for_training","come_here_for_evaluation","wave_for_evaluation","wave_for_training"]
    gestures_to_paint=["come_here_for_training","wave_for_training","come_here_for_evaluation","wave_for_evaluation"]
    gmm_models =["come_here_dim1.k3"]
    images = generate_images(gestures_to_paint)
    #for image_name, fig in images:
    #    fig.show()

        
