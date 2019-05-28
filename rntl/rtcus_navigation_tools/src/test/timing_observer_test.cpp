/*
 * timing_observer_test.cpp
 *
 *  Created on: Jun 19, 2012
 *      Author: geus
 */

#include <rtcus_navigation_tools_lib/timing_observer.h>

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_timing_node");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}



