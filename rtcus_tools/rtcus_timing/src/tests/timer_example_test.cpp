/*
 * timer_example_test.cpp
 *
 *  Created on: Jun 25, 2012
 *  Created on: Apr 10, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <rtcus_timing/timer.h>

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_node");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(Timer, ElementalTest)
{
  rtcus_timing::Timer timer;
  //anonymous chunk
  timer.start();
  ros::Duration(1.0).sleep();
  timer.stop();

  timer.start("second_chunk");
  ros::Duration(2.0).sleep();
  ros::Duration duration = timer.stop("second_chunk");
  ROS_INFO("duration of the second chunk: %lf",duration.toSec());

  timer.start("third_chunk");
  ros::Duration(3.0).sleep();
  timer.stop("third_chunk");

  timer.publish_data();
  ros::Duration(5.0).sleep();

}
