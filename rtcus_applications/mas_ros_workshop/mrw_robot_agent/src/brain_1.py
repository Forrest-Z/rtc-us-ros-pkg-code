#!/usr/bin/env python

import roslib; roslib.load_manifest('mrw_robot_agent')
import rospy

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class Brain:

    def __init__(self):
        rospy.init_node('brain')
        
        # Aqui publicar en el topic "move_base_simple/goal"
        
        
        # Aqui subscribirse en el topic "rviz_goal" utilizando 
	# la función callback "rviz_goal(self,pose)"
        
    def rviz_goal(self, pose):
	#Aquí coger el mensaje "pose" que viene desde el topic "rviz_goal" 
	#y reenviarlo al control del robot "move_base_simple/goal"
        
if __name__ == '__main__':

    Brain()
    rospy.spin()
