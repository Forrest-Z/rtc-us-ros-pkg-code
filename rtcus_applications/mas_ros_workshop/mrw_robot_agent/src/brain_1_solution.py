#!/usr/bin/env python

import roslib; roslib.load_manifest('mrw_robot_agent')
import rospy

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class Brain:

    def __init__(self):
        rospy.init_node('brain')
        
        # Publisher for move_base goal
        self.goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped)
        
        # Subscribers for possible goal location sources
        self.rviz_goal_subscriber = rospy.Subscriber('rviz_goal', PoseStamped, self.rviz_goal)
        
    def rviz_goal(self, pose):
        self.goal_publisher.publish(pose)
        
if __name__ == '__main__':

    Brain()

    rospy.spin()
