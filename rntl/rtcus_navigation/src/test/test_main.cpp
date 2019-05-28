/*
 * test_main.cpp
 *
 *  Created on: Apr 10, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 */

#include <gtest/gtest.h>
#include <ros/ros.h>

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ros::init(argc,argv,"test_node");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

