/*
 * world_perception_tests.cpp
 *
 *  Created on: Apr 10, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <mrpt_bridge/pose_conversions.h>
#include <gtest/gtest.h>
#include <rtcus_navigation/state_ports/default_state_port.h>
#include <rtcus_navigation/state_ports/adaptable_state_correction_port.h>
#include <ros/ros.h>
#include <ros/duration.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

TEST(StateCorrection, BasicInitialization)
{
  rtcus_navigation::state_ports::DefaultStatePort<nav_msgs::Odometry> gp2;

}

