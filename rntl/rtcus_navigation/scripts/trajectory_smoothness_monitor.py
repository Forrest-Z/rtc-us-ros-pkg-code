#!/usr/bin/python
import roslib
roslib.load_manifest("rtcus_navigation")
import rospy
import rtcus_navigation
import rtcus_navigation.monitors.ptc_prediction_error
from rtcus_navigation.monitors.smooth_kurvature_monitor import SmoothKurvatureMonitor

rospy.init_node("smooth_kurvature");
if __name__ == "__main__":    
    SmoothKurvatureMonitor()
    while not rospy.is_shutdown():
        rospy.sleep(0.001)