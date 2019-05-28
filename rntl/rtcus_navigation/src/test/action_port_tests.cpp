/*
 * world_perception_tests.cpp
 *
 *  Created on: Apr 10, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <gtest/gtest.h>
#include <rtcus_navigation/action_ports/ros_action_port.h>
#include <rtcus_stamp/stamped.h>
#include <geometry_msgs/Twist.h>

using namespace geometry_msgs;

TEST(ActionPorts, BasicInitialization)
{
  ros::NodeHandle test;
  printf("1\n");

  rtcus_navigation::action_ports::ROSActionPort<geometry_msgs::Twist> action_port;
  Twist cmd_vel;
  action_port.sendAction(cmd_vel, ros::Time::now());

}
