#!/usr/bin/python
import roslib
roslib.load_manifest("rtcus_navigation")
import rospy
import std_msgs.msg
import nav_msgs
from nav_msgs.msg import Odometry
import numpy
import rtcus_nav_msgs
import tf
import math

class SmoothKurvatureMonitor():
    def __init__(self):
        rospy.loginfo("Monitoring kurvature...")
        self.distance_sub_ = rospy.Subscriber("/odom", Odometry, self.callback);
        self.delta_kurvature_pub_ = rospy.Publisher("delta_kurvature", std_msgs.msg.Float32)
        self.last_alpha_kurv = None
        self.delta_kurv_buffer = []
        self.last_odom_msg = None
    
    def restart(self):
        '''This is called at the begining of each simulated experiment'''
        self.last_alpha_kurv = None
        self.delta_kurv_buffer = []
        self.last_odom_msg = None
        
    def callback(self, msg):
        '''This is called on each simulation step. It uses an odometry message received from the simulator'''
        #get the speed information
        current_twist = msg.twist.twist
        current_kurvature = 0;
        alpha_kurv = None    
        current_v = numpy.sqrt(current_twist.linear.x ** 2 + current_twist.linear.y ** 2)
        if(current_v > 0):
            alpha_kurv = math.atan2(current_v, current_twist.angular.z)
        else:
            alpha_kurv = self.last_alpha_kurv
            
        if(self.last_odom_msg!=None and self.last_alpha_kurv != None and alpha_kurv != None):
            #rospy.loginfo("last odom %s", str(self.last_odom_msg))
            #rospy.loginfo("odom %s",str(msg))
            
            dt = msg.header.stamp.to_sec() - self.last_odom_msg.header.stamp.to_sec()  
            delta_alpha_kurv = numpy.abs(self.last_alpha_kurv - alpha_kurv)
            while(delta_alpha_kurv > numpy.pi):
                delta_alpha_kurv -= numpy.pi
            delta_alpha_kurv /= dt
            outmsg = std_msgs.msg.Float32()
            outmsg.data = delta_alpha_kurv
            self.delta_kurvature_pub_.publish(outmsg)
            self.delta_kurv_buffer.append(delta_alpha_kurv)
            #rospy.loginfo("delta kurv %s %s", self.last_alpha_kurv,alpha_kurv)
                
        self.last_alpha_kurv=alpha_kurv
        self.last_odom_msg=msg