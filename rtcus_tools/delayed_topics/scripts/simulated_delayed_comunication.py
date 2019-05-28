#!/usr/bin/python
PKG = 'delayed_topics'
import roslib; 
roslib.load_manifest(PKG)
import rospy
import delayed_topics
from delayed_topics.delayed_topic import RosMsgsRandomDelaySimulationQueue, RosTfMappingDelaySimulation
import std_srvs.srv  

if __name__ == "__main__":
    rospy.init_node("delayed_msgs")

    topics_param_key = rospy.get_name() + '/delayed_topics'
    rospy.loginfo("Initializing simulation of delayed msgs")
    
    delayed_topics = []
    if rospy.has_param(topics_param_key):
        topics_config = rospy.get_param(topics_param_key)
        rospy.loginfo("Topics config detected:" + str(topics_config))
                
        for topic_config in topics_config:
            if topic_config.has_key("frame_format"):
                frame_format = topic_config["frame_format"]
            else:
                frame_format = "";
                
            simDelay = RosMsgsRandomDelaySimulationQueue(topic_config["input_topic_name"],
                                                    topic_config["output_topic_name"],
                                                    topic_config["topic_type"],
                                                    topic_config["delay_mean"],
                                                    topic_config["delay_std"],
                                                    frame_format,
                                                    topic_config["rebuild_timestamp"])
            delayed_topics.append(simDelay)
    

    if rospy.has_param(rospy.get_name() + '/tfMapping'):
        tfMappingConfig = rospy.get_param(rospy.get_name() + '/tfMapping')
        print "------------------"
        print tfMappingConfig
        simTfMapping = RosTfMappingDelaySimulation(tfMappingConfig["prefix"],
                                                 tfMappingConfig["fixed_frame"],
                                                 tfMappingConfig["delay_mean"],
                                                 tfMappingConfig["delay_std"])
        delayed_topics.append(simTfMapping)
        
        
        
    def reset_callback(req):
       global delayed_topics
       rospy.loginfo("RESET RECEIVED")
       for dt in delayed_topics:
           dt.reset()
       return std_srvs.srv.EmptyResponse()
           
    def stop_callback(req):
       global delayed_topics
       rospy.loginfo("STOP RECEIVED")
       for dt in delayed_topics:
           dt.stop()
       return std_srvs.srv.EmptyResponse()
           
    def start_callback(req):
       global delayed_topics
       rospy.loginfo("START RECEIVED")
       for dt in delayed_topics:
           dt.run()
       return std_srvs.srv.EmptyResponse()
        
    reset_service = s = rospy.Service(rospy.resolve_name("~") + 'reset', std_srvs.srv.Empty, reset_callback)
    stop_service = s = rospy.Service(rospy.resolve_name("~") + 'stop', std_srvs.srv.Empty, stop_callback)
    start_service = s = rospy.Service(rospy.resolve_name("~") + 'start', std_srvs.srv.Empty, start_callback)
    
    start_callback(None)
        
    rospy.spin();
