#!/usr/bin/python
import roslib
roslib.load_manifest("rtcus_navigation")
import rospy
import rtcus_navigation
import rtcus_navigation.monitors.ptc_prediction_error
from rtcus_navigation.monitors.ptc_prediction_error import PredictionErrorStatsMonitor
        
rospy.init_node("prediction_error_node");
if __name__ == "__main__":    
    PredictionErrorStatsMonitor()
    while not rospy.is_shutdown():
        rospy.sleep(0.001)
                
  
