/*
 * world_perception_tests.cpp
 *
 *  Created on: Apr 10, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <mrpt_bridge/pose_conversions.h>
#include <gtest/gtest.h>
#include <rtcus_navigation/goal_ports/default_goal_port.h>
#include <rtcus_navigation/goal_ports/adaptable_goal_port.h>
#include <ros/ros.h>
#include <ros/duration.h>
#include <geometry_msgs/Pose.h>

TEST(GoalPorts, BasicInitialization)
{

  rtcus_navigation::goal_ports::DefaultGoalPort<geometry_msgs::Pose> gp;

}
