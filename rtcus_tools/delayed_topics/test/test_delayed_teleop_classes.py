#!/usr/bin/env python
import numpy
from numpy import *
PKG = 'delayed_topics'
import roslib; 
roslib.load_manifest(PKG)
import rospy
import sys
import unittest
import heapq
import std_msgs
import threading
from heapq import *

import delayed_topics
from delayed_topics.delayed_topics import RosMsgsRandomDelaySimulationQueue

class delayed_topics_tests(unittest.TestCase):

    def tearDown(self):
        self.exit = True
        return
    
    def background_topic_publisher(self):
        i = 0;
        while not rospy.is_shutdown() and not self.exitd:
            self.input_publisher.publish(std_msgs.msg.Int64(data=i))
            i += 1
            rospy.sleep(self.publish_period)
        
    def setUp(self):
        rospy.init_node("test_delayed_comunications")
        self.input_publisher = rospy.Publisher("input", std_msgs.msg.Int64)
        self.exitd = False
        self.publish_period = 0.0001
        self.background_task = threading.Thread(target=self.background_topic_publisher)
        self.background_task.start();
        
    def publish_callback(self, msg, delay_info, current_time):
        print "message [%s] resent expected-sent-stamp [%s] - current time [%s]" % (msg.data, delay_info, current_time)
        self.assertGreaterEqual(delay_info, self.last_stamp)
        self.last_stamp = delay_info
        
    def test_basic(self):
        delay_mean = 0.01 * self.publish_period #assert that buffer is not empty
        delay_std = 0.01
        q = RosMsgsRandomDelaySimulationQueue("input", "output", "std_msgs/Int64", delay_mean, delay_std, "%s")
        self.last_stamp = 0.0
        print "START TIME: %lf" % self.last_stamp
        q.onPublishObservers.append(self.publish_callback) 
        q.run()
        
        print "sleeping the main test thread"
        rospy.sleep(5)
        print "exiting"
        q.exit = True
        ordered = []
        while q.delayed_msg_queue:
            ordered.append(heappop(q.delayed_msg_queue))
        
        before = ordered[0]
        for current in ordered[1:]:
            self.assertGreaterEqual(current[0], before[0])
            before = current
        
        
if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'delayed_topics_tests', delayed_teleop_tests)
