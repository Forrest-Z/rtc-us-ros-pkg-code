#!/usr/bin/python
import threading
import numpy
import time
import roslib
roslib.load_manifest('delayed_teleop')
import rospy
import sensor_msgs
import sensor_msgs.msg
import geometry_msgs
import geometry_msgs.msg
import nav_msgs
import nav_msgs.msg
import tf
import tf.msg
import sys
import rosgraph_msgs
import rosgraph_msgs.msg

class GoalCorrector:
    def publishExpectedGoal(self,goal_msg):
        #time travel to past, and get the position
        #then publish the new corrected goal
        (trans,rot)=self.listener.lookupTransform('/base_link', '/odom', rospy.Time(3))
        print trans,rot
        pass
    
    def inputGoalMsgCallback(self,goal_msg):
        print "goal_callback"
        publishExpectedGoal(goal_msg)
    

    def __init__(self):    
        self.listener = tf.TransformListener()
        self.input_topic = rospy.Subscriber("input_goal_pose", geometry_msgs.msg.PoseStamped, self.inputGoalMsgCallback)
        self.output_topic = rospy.Publisher("output_goal_pose", geometry_msgs.msg.PoseStamped)

rospy.init_node("goal_corrector")
gcorrector=GoalCorrector()        
rospy.spin();