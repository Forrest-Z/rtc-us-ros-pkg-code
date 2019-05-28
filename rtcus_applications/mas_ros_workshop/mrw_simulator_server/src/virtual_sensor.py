#!/usr/bin/env python

import roslib; roslib.load_manifest('mrw_simulator_server')
import rospy

from math import sqrt

from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

fire_positions = [Point(20, 20, 0)]
current_fire = fire_positions[0]

class Sensor:

    state = False
    
    def __init__(self, number):
        self.number = number
        
        self.pub = rospy.Publisher('/robot_' + str(number) + '/smoke_sensor', Bool)
        rospy.Subscriber('/robot_' + str(number) + '/base_pose_ground_truth', Odometry, self.callback)
        
    def callback(self, data):
        position = data.pose.pose.position
        
        dx = (current_fire.x - position.x) ** 2
        dy = (current_fire.y - position.y) ** 2
        dz = (current_fire.z - position.z) ** 2
        
        distance = sqrt(dx + dy + dz)
        
        newstate = (distance < 3)
        
        if self.state ^ newstate:
            self.pub.publish(Bool(newstate))
        
        self.state = newstate

def main():
    sensors = []

    rospy.init_node('virtual_sensor')

    for i in range(9):
        sensors.append(Sensor(i))

    rospy.spin()

if __name__ == '__main__':
    main()
