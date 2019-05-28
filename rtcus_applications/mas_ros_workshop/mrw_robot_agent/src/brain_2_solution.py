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
        
        # Publisher for fire location
        self.fire_location_publisher = rospy.Publisher('/fire_location', PoseStamped)

        # Subscribers for onboard sensors
        self.sensor_subscriber = rospy.Subscriber('smoke_sensor', Bool, self.smoke_sensor)
        self.odometry_subscriber = rospy.Subscriber('odom', Odometry, self.odometry_callback)

        # Subscribers for possible goal location sources
        self.rviz_goal_subscriber = rospy.Subscriber('rviz_goal', PoseStamped, self.rviz_goal)
        self.fire_goal_subscriber = rospy.Subscriber('/fire_location', PoseStamped, self.fire_goal)
        
    def rviz_goal(self, pose):
        self.goal_publisher.publish(pose)
        
    def fire_goal(self, pose):
        # Ignore rviz goal commands
        self.rviz_goal_subscriber.unregister()
        # Go to fire location
        self.goal_publisher.publish(pose)
        # Unsubscribe from fire location topic
        self.fire_goal_subscriber.unregister()
        
    def smoke_sensor(self, fire_detected):
        if fire_detected.data:
            # Stop the robot
            self.goal_publisher.publish(self.pose_stamped)
            
            # Notify all other robots
            self.fire_location_publisher.publish(self.pose_stamped)
            
    def odometry_callback(self, data):
        self.pose_stamped = PoseStamped()
        self.pose_stamped.header.stamp = rospy.Time.now()
        self.pose_stamped.header.frame_id = data.header.frame_id
        self.pose_stamped.pose = data.pose.pose
        
if __name__ == '__main__':

    Brain()

    rospy.spin()
