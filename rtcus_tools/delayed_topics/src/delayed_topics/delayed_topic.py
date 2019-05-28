#!/usr/bin/python
import threading
import numpy
import time
import roslib
from heapq import heappush, heappop
roslib.load_manifest('delayed_topics')
import delayed_topics.cfg
from delayed_topics.cfg import DelayedTopicConfig

import rospy
import sensor_msgs
import sensor_msgs.msg
import geometry_msgs
import geometry_msgs.msg
import nav_msgs
import nav_msgs.msg
import std_msgs
import std_msgs.msg
import rtcus_nav_msgs
import rtcus_nav_msgs.msg
import tf
import tf.msg
import sys
import rosgraph_msgs
import rosgraph_msgs.msg
import heapq
from heapq import *
from dynamic_reconfigure.server import Server

current_rostime = rospy.Time()
def clockUpdate(time_msg):
    global current_rostime
    current_rostime = rospy.Time(secs=time_msg.clock.secs, nsecs=time_msg.clock.nsecs)
    #print "current_ros_time:"+str(current_rostime)

clock_sub = rospy.Subscriber("/clock", rosgraph_msgs.msg.Clock, clockUpdate)

def getCurrentRosTime_sim():
    return current_rostime

def getCurrentRosTime_rt():
        return rospy.Time.now()
    #global current_rostime
    

class TopicDelayTask(threading.Thread):
    def stop(self):
        return


class RosMsgsRandomDelaySimulation(object):    
    def __init__(self, input_topic_name, output_topic_name, topicTypeName, delay_mean, delay_std, frame_format, rebuild_stamp=True):
        self.input_topic_name = input_topic_name
        self.output_topic_name = output_topic_name
        self.rebuild_stamp = rebuild_stamp;
    
        if rospy.has_param("/sim_time") and rospy.get_param("/sim_time")==True:
            self.getCurrentRosTime= lambda: getCurrentRosTime_sim()
        else:
            self.getCurrentRosTime= lambda: getCurrentRosTime_rt()
            
        
        pkg_name, msg_name = tuple(topicTypeName.split("/"))
        rospy.loginfo("loading package: %s" % pkg_name)
        __import__(str(pkg_name))
        __import__(str(pkg_name) + ".msg")
        __import__(str(pkg_name) + ".msg", msg_name)
        self.topicType = eval(pkg_name + ".msg." + msg_name)
        
        self.input_topic = rospy.Subscriber(self.input_topic_name, self.topicType, self.sensorMsgCallback)
        self.output_topic = rospy.Publisher(self.output_topic_name, self.topicType)
        rospy.loginfo("initializating, delay mean: %s" % delay_mean)
        self.delay_mean = delay_mean
        rospy.set_param(rospy.resolve_name("~" + "topic_" + self.input_topic_name + "_config") + "/delay_mean", delay_mean)
        
        self.delay_std = delay_std
        rospy.set_param(rospy.resolve_name("~" + "topic_" + self.input_topic_name + "_config") + "/delay_std", delay_std)
        
        self.frame_format = frame_format
        self.srv = Server(DelayedTopicConfig, self.config_server_callback, "topic_" + input_topic_name + "_config")
        
    def reset(self):
        return
    
    def stop(self):
        return
        
    def config_server_callback(self, config, level):
        rospy.logwarn("the delay mean value is %lf", float(rospy.get_param(rospy.resolve_name("~" + "topic_" + self.input_topic_name + "_config") + "/delay_mean", self.delay_mean)))
        self.delay_mean = config.delay_mean
        self.delay_std = config.delay_std
        return config
            
    def getRandom(self):
        return numpy.random.normal(self.delay_mean, self.delay_std);
    
    def resendDelayedSensorMsg(self, msg, delay_info):
        if(hasattr(msg, "header")):
            if msg.header.frame_id != None and msg.header.frame_id != "":
                if msg.header.frame_id[0] == "/":
                    frame_subname = msg.header.frame_id[1:]
                else:
                    frame_subname = msg.header.frame_id
            else:
                frame_subname = ""
                
                
                
            if self.frame_format != "":
               try:
                    frame_name = self.frame_format % frame_subname
               except:
                    frame_name = frame_subname
            else:
                frame_name = msg.header.frame_id
                
            msg.header.frame_id = frame_name
            if self.rebuild_stamp:
                msg.header.stamp = self.getCurrentRosTime()
                
        self.output_topic.publish(msg)
    
    def sensorMsgCallback(self, msg):
        delay = self.getRandom()
        #if hasattr(msg,"header"):
        #    print "just received msg stamp: "+str(msg.header.stamp)
            
        task = threading.Timer(delay, lambda msg_copy=msg, delay_info=delay: self.resendDelayedSensorMsg(msg_copy, delay_info))
        task.start()
        

class RosMsgsRandomDelaySimulationQueue(RosMsgsRandomDelaySimulation):
    def __init__(self, input_topic_name, output_topic_name, topicTypeName, delay_mean, delay_std, frame_format, rebuild_stamp=True):
        self.is_stopped = True
        RosMsgsRandomDelaySimulation.__init__(self, input_topic_name, output_topic_name, topicTypeName, delay_mean, delay_std, frame_format, rebuild_stamp)
        rospy.loginfo("creating delayed from topic '%s' to topic '%s'" % (input_topic_name, output_topic_name))
        self.delayed_msg_queue = []
        #Thread
        self.task = None

        
        #queue
        self.queue_lock = threading.Semaphore()
        self.task_lock = threading.Semaphore()
        #Time
        self.startTime = self.getCurrentRosTime()
        
        #parameters
        self.minimum_delay_gap = rospy.Duration(0.001);
        self.max_queue_size = 5000
                
        #for tcp ordered receiving
        self.last_expected_stamp = self.startTime
        self.buffer_size_info_pub = rospy.Publisher("~/buffer_size", std_msgs.msg.Int64)
        
        #event
        self.onPublishObservers = []
        
    
    def reset(self):
        rospy.loginfo("RESETING delayed message buffer data")
        self.queue_lock.acquire()
        self.delayed_msg_queue = []
        self.is_stopped = True
        self.queue_lock.release()
        return
    
    def run(self):
        if self.is_stopped:
            rospy.loginfo("START the rostopic delay from topic '%s' to topic '%s'" % (self.input_topic_name, self.output_topic_name))
            self.task = TopicDelayTask(target=self.periodic_task)
            self.is_stopped = False
            self.task.start();
        else:
            rospy.loginfo("The delay topic thread task is already started. No application has been done.")
        
    def stop(self):
        self.task_lock.acquire(True)
        rospy.loginfo("STOPPING for topic '%s'", self.input_topic_name)
        if self.task != None:
            self.is_stopped = True
            self.task.stop()
        else:
            rospy.loginfo("The delay topic is not running now. STOP order ignored.")
        
        self.task_lock.release()
        
    def sensorMsgCallback(self, msg):
        if self.is_stopped:
            return
        
        if not hasattr(self, "delayed_msg_queue"):
            rospy.logerr("topic callback. but there was a problem during the topic delay initialization.")
            return;
        
        if(len(self.delayed_msg_queue) > self.max_queue_size):
            #Node Saturated DROOOP
            rospy.logwarn("topic callback. The topic buffer is saturated (len %d of a max len of %d) resending time mean (%lf)" % (int(len(self.delayed_msg_queue)), int(self.max_queue_size), float(self.delay_mean)))
            return            
                
        self.queue_lock.acquire()
        #assert SORTED RESEND ORDER with stamps
        random_delay = rospy.Duration(abs(self.getRandom()))
        current_time = self.getCurrentRosTime()
        expected_stamp = current_time
        expected_stamp += random_delay 
        if(expected_stamp < self.last_expected_stamp):
            expected_stamp = self.last_expected_stamp + self.minimum_delay_gap
        self.last_expected_stamp = expected_stamp
        #rospy.loginfo("new message at [%s], it will be sent at [%s] -> %s", current_time.to_sec(), expected_stamp.to_sec(), str(msg))
            
        #push message for later
        future_msg = (expected_stamp, msg)
        heappush(self.delayed_msg_queue, future_msg)
        self.queue_lock.release()
    
    def periodic_task(self):
        rospy.loginfo("delay topics - task started")
        while not rospy.is_shutdown() and not self.is_stopped:
            self.task_lock.acquire(True)
            local_queue = [] #it will contain msgs ready to be sent
            self.queue_lock.acquire()
            #1 - enque in the local que all ready messages
            if len(self.delayed_msg_queue) > 0:
                earliest_msg = heappop(self.delayed_msg_queue)
                current_time = self.getCurrentRosTime()
                #print "quesize[%s], check earlier? [%s] > [%s] -> %s"%(len(self.delayed_msg_queue),earliest_msg[0].to_sec(),current_time.to_sec(),earliest_msg[0]>current_time)
                while earliest_msg[0] < current_time and len(self.delayed_msg_queue) > 0:
                    local_queue.append(earliest_msg)
                    earliest_msg = heappop(self.delayed_msg_queue)
                if earliest_msg[0] < current_time: #handle the last poped msg
                    local_queue.append(earliest_msg)
                else: #return back to the queue
                    heappush(self.delayed_msg_queue, earliest_msg)

            self.queue_lock.release()
            #2 - resend all local_queue_msgs        
            for msg_data in local_queue:
                self.resendDelayedSensorMsg(msg_data[1], msg_data[0])
            
            if len(local_queue) == 0:
                time.sleep(self.minimum_delay_gap.to_sec())
            else:
                self.buffer_size_info_pub.publish(std_msgs.msg.Int64(data=len(self.delayed_msg_queue)))
            self.task_lock.release()
        rospy.loginfo("delay topics - task finished")
                
            
            
    def resendDelayedSensorMsg(self, msg, delay_info):
        RosMsgsRandomDelaySimulation.resendDelayedSensorMsg(self, msg, delay_info)
        current_time = self.getCurrentRosTime()
        for callback in self.onPublishObservers:
            callback(msg, delay_info.to_sec(), current_time.to_sec())
        
        
class RosTfMappingDelaySimulation(RosMsgsRandomDelaySimulationQueue):
    def __init__(self, prefix, fixed_frame, delay_mean, delay_std):
        RosMsgsRandomDelaySimulationQueue.__init__(self, "/tf", "/tf", "tf/tfMessage", delay_mean, delay_std, "")
        self.prefix = prefix
        self.fixed_frame = fixed_frame
    
    def sensorMsgCallback(self, msg):
        if not hasattr(self, "prefix"):
            rospy.logerr("topic callback. but there was a problem during the topic tf delay initialization.")
            return;
        
        #filter resen messages
        for i in xrange(len(msg.transforms)):
            frame_id = msg.transforms[i].header.frame_id;
            child_frame_id = msg.transforms[i].child_frame_id;
            if  frame_id.startswith(self.prefix) or child_frame_id.startswith(self.prefix):
                return 
            
        RosMsgsRandomDelaySimulationQueue.sensorMsgCallback(self, msg)
        
    def resendDelayedSensorMsg(self, msg, delay_info):

        for i in xrange(len(msg.transforms)):
            frame_id = msg.transforms[i].header.frame_id;
            child_frame_id = msg.transforms[i].child_frame_id;
            oldStamp = msg.transforms[i].header.stamp
            newStamp = self.getCurrentRosTime()
            auch = False
            if(newStamp < oldStamp):
                newStamp = oldStamp
                auch = True
                
            if not frame_id.startswith(self.prefix) and frame_id != self.fixed_frame:
                msg.transforms[i].header.frame_id = self.prefix + frame_id[1:]
                #print "tf restamped [%s] old [%s] new [%s][auch:%s]"%(child_frame_id,oldStamp,newStamp,auch)
                msg.transforms[i].header.stamp = newStamp#rospy.Duration(delay) # trick must understand
                
            if not child_frame_id.startswith(self.prefix) and child_frame_id != self.fixed_frame:
                msg.transforms[i].child_frame_id = self.prefix + child_frame_id[1:]
                msg.transforms[i].header.stamp = self.getCurrentRosTime()#rospy.Duration(delay)  # trick must understand
                
        self.output_topic.publish(msg)
        
