/*
 *  Created on: April 13, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */
#include <rtcus_dwa/simple_dwa_ros.h>
#include <rtcus_navigation/navigation_node.h>
#include <rtcus_assert/rtcus_assert.h>
#include <rtcus_navigation_tools/timing_observer.h>

using namespace rtcus_dwa;

int main(int argc, char** argv)
{

  ros::init(argc, argv, "navigation_node");
  RTCUS_ASSERT_CATCH_SIGSEGV();

  ROS_INFO("DWA Module. Creating dwa-navigation node...");

  DwaLocalPlanner::NavigationNodePtr nn;
  nn = rtcus_navigation::NavigationNodeFactory<DwaLocalPlanner>::create_navigation_node();

  ROS_INFO("creating timing wrapper");
  rtcus_navigation_tools::TimeObserver<DynamicState2D, pcl::PointCloud<pcl::PointXY>, Twist2D, PointXY> timing(nn);

  nn->start();
  ros::spin();
  return 0;
}

