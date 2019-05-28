#!/usr/bin/env python
import os
import roslib
roslib.load_manifest('rtcus_kinect_gestures')
import rospy
import rosparam
import rosnode
import LbD
from LbD.GestureEntry import *
import rosbag
import roslaunch
import param_initialization
import PyKDL
import tf
import subprocess

from GesturesDatabaseFacade import BagDatabase, GestureDatabase
from optparse import OptionParser
            
def gesture_pull_tf_messages(bag_name, demo_index, gesture, gesture_params, listener):
    """
    @summary: This function listens to the tf topic to get the tf frames states. Then pushes the interesting frames to the gesture object to make a trajectory.
    """
    period_time = 1.0 / gesture_params.SAMPLING_FREQUENCY
    start_time = -1.0
    end_time = sys.maxint
    transformations_count = 0
    #flag for error checking
    last_transformation_count = 0
    recorded_messages = []
    sample = 0;
    before_latest_common_time = [numpy.double(-1.0) for _ in xrange(gesture.frame_count)]

    if gesture_params.SECONDS_TO_START_RECORDING > 0:
        rospy.loginfo("waiting %f seconds (parameter 'seconds to start recording'", gesture_params.SECONDS_TO_START_RECORDING)
        rospy.sleep(gesture_params.SECONDS_TO_START_RECORDING)
        
    latest_commontime = rospy.Time.to_sec(rospy.Time.now())
    same_time = 0
    end = False
    rospy.loginfo("starting tf messages pulling")
    while not end and rospy.Time.to_sec(rospy.Time.now()) < end_time:
        if start_time != -1.0:
            rospy.logdebug("currtime: %s", str(rospy.Time.to_sec(rospy.Time.now()) - start_time))
        else:
            rospy.logdebug("currtime: %s, endtime: %s", str(rospy.Time.to_sec(rospy.Time.now())), str(end_time))
        
        start_catching = rospy.Time.to_sec(rospy.Time.now())
        for fr in xrange(gesture.frame_count):
            (fixed_frame, target_frame, variables) = gesture.get_frame_config(fr)
            try:
                #if listener.canTransform(target_frame,fixed_frame,rospy.Time.now()):
                
                try:
                    latest_commontime = numpy.double(listener.getLatestCommonTime(target_frame, fixed_frame).to_sec())
                except:
                    rospy.logdebug("latest common time from tf buffer not got... something bad in the bag raw file")
                    same_time += 1
                    continue
                    
                #BUG OF ROS!!! strs!!
                if str(before_latest_common_time[fr]) == str(latest_commontime):
                    same_time += 1
                    continue
                else:
                    same_time = 0
                    
                before_latest_common_time[fr] = latest_commontime
                                    
                (trans, rot) = listener.lookupTransform(fixed_frame, target_frame , rospy.Time(0.0))
                ttime = rospy.Time.to_sec(rospy.Time.now())
                rot_rpy = PyKDL.Rotation.Quaternion(rot[0], rot[1], rot[2], rot[3]).GetRPY()
                
                if start_time == -1.0:
                    rospy.logdebug("First tf message received")
                    now = ttime
                    start_time = now
                    end_time = now + gesture_params.TIME_PER_DEMONSTRATION
                    rospy.logdebug("current time is: %s secs, recording until: %s secs", now, end_time)
                    rospy.logdebug("first entry: %s at time", str(now))
                    
                #push tf message
                entry = (fr, trans, rot_rpy, demo_index, ttime - start_time)
                recorded_messages.append(entry)
                transformations_count = transformations_count + 1
            except Exception, ex:
                rospy.loginfo("exception %s", str(ex))
                if start_time != -1.0 and last_transformation_count != transformations_count:
                    #simple transform error 
                    last_transformation_count = transformations_count 
                    rospy.logdebug("[error in transformation %d] %s", transformations_count, str(ex))
            
        #wait a period
        while (rospy.Time.to_sec(rospy.Time.now()) - start_catching) / period_time < 0.95:
            #sleep a very small piece of time to reach the sampling period
            rospy.sleep(0.05 / gesture_params.SAMPLING_FREQUENCY)
        
        #check if no tf messages during the half of time
        repeated_times = float(same_time) / float(gesture.frame_count) 
        #rospy.loginfo("times with no new message: (%d,number of frames: %d)->%d",same_time,gesture.frame_count,repeated_times)
        if repeated_times > 10 :
            rospy.logdebug ("time without a tf message--> %f seconds", repeated_times / gesture_params.SAMPLING_FREQUENCY)
            
        if repeated_times / gesture_params.SAMPLING_FREQUENCY > gesture_params.TIME_PER_DEMONSTRATION * 2.0:
            rospy.logerr("no frame received during too much time, exiting")
            end = True
            
        sample += 1
                
    #check recorded data coherence
    #if incoherent return None
    if len(recorded_messages) <= 2:
        rospy.logwarn("[DEMO %d DISCARDED] not enough transforms (probably end of bagfile)|| bag_file:%s ", demo_index, bag_name)
        return None

    #recording time with a 20% of margin
    end_time = recorded_messages[-1][-1]
    if end_time * 1.2 <= gesture_params.TIME_PER_DEMONSTRATION:
        rospy.logwarn("[DEMO %d DISCARDED] recorded time not enough huge, just: %f seconds when at least %f seconds needed || bag_file:%s ", demo_index, end_time, gesture_params.TIME_PER_DEMONSTRATION, bag_name)
        return None
    
    #number of samples with a 20% of margin
    expected_samples = gesture_params.TIME_PER_DEMONSTRATION / gesture_params.SAMPLING_FREQUENCY
    if len(recorded_messages) * 1.2 < expected_samples:
        rospy.logwarn("[DEMO %d DISCARDED] not enough sampling frequency done. len(samples):%f , expected samples:%f || bag_file:%s ", demo_index, len(recorded_messages), expected_samples, bag_name)
        return None
    
    if transformations_count == 0:
        rospy.logwarn("[DEMO %d DISCARDED] No transformations catched|| bag_file:%s ", demo_index, bag_name)
        return None
    
    rospy.loginfo ("subdemo processed || number of entries: %s", len(recorded_messages))
    return recorded_messages
                    
def create_gesture_from_bags(gesture_bag_folder_name, gesture_bag_files, gesture_params, listener):
    """
    @summary: This function open the interested bagfiles, start the bag play process and start the gesture variable trajectory recording
    """
    gesture_name = gesture_params.GESTURE_NAME
    rospy.loginfo("creating gesture - %s: %s" % (gesture_name, gesture_bag_folder_name))             
    gesture = GestureEntry(gesture_name, gesture_params.FRAMES, gesture_params.SAMPLING_FREQUENCY, gesture_params.TIME_PER_DEMONSTRATION)  
    rospy.loginfo("Gesture creation configuration (%s): %s", gesture_bag_folder_name, [gesture.get_frame_config(i) for i in xrange(gesture.frame_count)])
    demonstrations_names = []
    recorded_sub_demos = []
    demo_index = 0
    for (current_dem, (gesture_bag_file, bag_name)) in enumerate(gesture_bag_files):
        rospy.loginfo("==== PROCESSING BAG FILE %d ====", current_dem)
        node_name = ("%s_%s") % ("".join([ c for c in gesture_bag_folder_name if c.isalpha()]), str(current_dem))
        bag_bin_path = roslib.packages.find_node("rosbag", "rosbag")
        arguments = ["play", gesture_bag_file, "name:=" + (node_name), "tf:=tf_bag"]
        #rospy.loginfo("environment: %s",roslib.packages.os.environ)
	#start play bag file
        bag_process = subprocess.Popen([bag_bin_path] + arguments, env=roslib.packages.os.environ, shell=False)
        rospy.logdebug("new process opened: %s %s. Arguments: %s", str(bag_process), str(bag_process.pid), str(arguments))
        rospy.loginfo("processing bag file data: %s", bag_name)
        rospy.logdebug("Time per demonstration: %s secs", gesture_params.TIME_PER_DEMONSTRATION)
        rospy.logdebug("Waiting first tf message")
    
        subdemo_count = 0
        recorded_tf_messages = None
                
        while subdemo_count == 0 or recorded_tf_messages != None:
            rospy.loginfo("[DEMO %d] (%d / bagfile: %s)", demo_index, subdemo_count, bag_name)
            recorded_tf_messages = gesture_pull_tf_messages(bag_name, demo_index, gesture, gesture_params, listener)
            
            if recorded_tf_messages == None:
               break
           
            recorded_sub_demos.append(recorded_tf_messages)
            demonstrations_names.append(str(bag_name) + "_subdemochunk_" + str(demo_index))
            demo_index += 1
            subdemo_count += 1
            print "------"
            rospy.loginfo("waiting %f seconds (parameter 'wait_seconds')", gesture_params.WAIT_SECONDS)
            rospy.sleep(gesture_params.WAIT_SECONDS)
            print "------"

        
        rospy.loginfo("End of bagfile. Dispossing bag process if still open.")    
        #--------------disposing bag subprocess ---------------------
        rosnode.kill_nodes(node_name)
        rospy.logdebug("node killed")
        if bag_process.poll():
            rospy.logdebug("bag process self terminated\n")
        else:
            rospy.logdebug("bag process still alive, killing\n")
            try:    
                bag_process.terminate()
            except:
                continue
            
        bag_process.wait()
        rospy.sleep(2)
        #sys.stdin.readline()
        #-----------------------------------------------------
        
    #pushing messages to the gesture datastructure
    
    rospy.loginfo("== ALL BAG REPRODUCTION FILES PROCESED === Total of subdemos got: %d", len(recorded_sub_demos))
    rospy.loginfo("storing data in the gesture object")
    for subdemo_samples in recorded_sub_demos:
        for (frame_index, trans, rpy, demonstration, time) in subdemo_samples:
            if demonstration == gesture.demonstration_count:
                gesture.increment_demo_count()
            gesture.push_frame_data_from_tf(frame_index, trans, rpy, demonstration, time)
    gesture.demonstrations_names = demonstrations_names
    gesture.mark_gesture_defined()
    
    try:
        rospy.loginfo("marking gesture as defined and checking coherence")
        #check full coherence
        gesture.mark_gesture_defined()
    except Exception, e:
        rospy.logfatal("Incoherent gesture database created: %s", str(e.message))
        exit()
                      

    return gesture
            
def process_gesture_simple_gesture(gesture_bag_folder_name, bags_db, gesture_params, listener, split, gesture_db):
        rospy.loginfo("------------ gesture folder: %s ----------------", gesture_bag_folder_name)
        gesture_bags = bags_db.get_gesture_bags(gesture_bag_folder_name)
        rospy.loginfo("gesture files: ")
        for gn in gesture_bags:
            rospy.loginfo("file: " + str(gn[0]))
        print "-----------------"    
        (config_file, config) = bags_db.get_gesture_config(gesture_bag_folder_name)
        
        if config_file != None:
            gesture_params.load_config_file(config_file)
            gesture_params.refresh()
            
        gesture = create_gesture_from_bags(gesture_bag_folder_name, gesture_bags, gesture_params, listener)
        
        if split != "False":
            gesture_split_names = [gesture_bag_folder_name + "_for_training", gesture_bag_folder_name + "_for_evaluation"]
            new_gestures = gesture.split(gesture_split_names)
            for i, new_gesture in enumerate(new_gestures):
                gesture_db.save_gesture_entry(gesture_split_names[i], new_gesture)
        else:
            gesture_db.save_gesture_entry(gesture_bag_folder_name, gesture)
            
        gesture_params.clear_parameters()
        gesture_params.reload_root_config_files()
        rospy.loginfo("gesture processed: %s", gesture_bag_folder_name)
            
def tfcallback(tfmessage):
    for transf in tfmessage.transforms:
        transf.header.stamp = rospy.Time.now()
        transf.header.frame_id = transf.header.frame_id.replace("2", "1")
        transf.child_frame_id = transf.child_frame_id.replace("2", "1")
        #rospy.loginfo("subs: %s",transf.child_frame_id)
    tfpublisher.publish(tfmessage)


if __name__ == '__main__':
    
    #WARNING THIS CODE WILL NOT WORK IF THE PATH ENVIRONMENT DOES NOT INCLUDE THE RTCUS_KINECT_PACKAGE AND ITS NOT ONLY VALID THE SHELL
    from optparse import OptionParser
    parser = OptionParser("%prog [ opts ]\nThis script processes sets of gesture bags to gesture.yaml files ")
    parser.add_option("-d", "--directory", default="", dest="directory", help="directory database")
    parser.add_option("-g", "--gesture-bag-folder", default="", dest="gesture_bag_folder", help="it defines the name of the folder where bags gestures are stored name and the generated gesture_entry")
    parser.add_option("-s", "--split", default="False", dest="split", help="This is usefull to generate two gestures entries for evaluation and estries. In some sense it will define the filename of the different gesture entries generated, typically {gesture_name}_for_training.gesture.yaml and {gesture_name}_for_evalutation.gesture.yaml")
    parser.add_option("-l", "--loglevel", default="INFO", dest="loglevel", help="loglevel \"INFO\" (default) or \"DEBUG\"")
    (options, args) = parser.parse_args()
    
    
    if options.loglevel == "DEBUG":
        loglevel = rospy.DEBUG
        rospy.loginfo("DEBUG logging mode")
    else:
        loglevel = rospy.INFO
        
    rospy.init_node ('batch_gesture_entries_from_bags')
    
    bags_db = BagDatabase()
    gesture_db = GestureDatabase()
    
    if options.directory != "": 
        if os.path.exists(options.directory) and not os.path.isfile(options.directory):
            bags_db.BAGS_DIRECTORY = options.directory
        else:
            rospy.logfatal("Incorrect directory. exiting.")
            exit(-1)
        
    rospy.set_param("sim_time", True)
    #files on the directory root are modeled independently
    rospy.loginfo("input bag dir: %s", bags_db.BAGS_DIRECTORY)
    #selecting config files
    
    root_config_files = bags_db.get_root_config_files()
    rospy.loginfo("loading global configuration for gestures")
    gesture_params = param_initialization.GestureParams(root_config_files)
    gesture_params.refresh()

    rospy.loginfo("configuring tf tunnel")
    tfpublisher = rospy.Publisher("tf", tf.msg.tfMessage)
    tfproxy = rospy.Subscriber("tf_bag", tf.msg.tfMessage, tfcallback)
    listener = tf.TransformListener()
    rospy.loginfo("split gesture:" + str(options.split))
        
    if options.gesture_bag_folder != "":
        #process just one specified gesture
        process_gesture_simple_gesture(options.gesture_bag_folder, bags_db, gesture_params, listener, options.split, gesture_db)
        exit()
    else:
        #process all gesture files in the root folder
        
        root_gesture_names = bags_db.get_uncategorized_gesture_bag_names()
        for gesture_name in root_gesture_names:
            process_gesture_simple_gesture(gesture_name, bags_db, gesture_params, listener, options.split, gesture_db)
        
        gesture_names = bags_db.get_gesture_entries_names()
        for gesture_name in gesture_names:
            process_gesture_simple_gesture(gesture_name, bags_db, gesture_params, listener, options.split, gesture_db)
        
        
        
    
    
    
            

