/*
 * state_composer.cpp
 *
 *  Created on: Jun 13, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <mrpt_bridge/mrpt_bridge.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <rtcus_compositions/state_composer.h>
#include <rtcus_nav_msgs/DynamicState2D.h>

using namespace rtcus_nav_msgs;

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(Compositions, dynamicCompositions)
{
  DynamicState2D state, result;
  DynamicState2D transform;

  state.pose.x = 1.0;
  state.pose.y = 0;
  state.pose.phi = 0;
  state.twist.angular = 1.0;
  state.twist.linear = 1.0;
  state.twist.lateral = 0.0;

  transform.pose.x = 10;
  transform.pose.y = 10;
  transform.pose.phi = 1.0;

  transform.twist.angular = -1.0;
  transform.twist.linear = 5.0;

  //rtcus_compositions::StateComposer::compose(state, transform, result);
}

