/*
 * shared_control_dwa_move_base.cpp
 *
 *  Created on: Oct 25, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_shared_dwa/shared_dwa_navigation_planner.h>
#include <rtcus_navigation/navigation_node.h>
#include <rtcus_assert/rtcus_assert.h>
#include <rtcus_navigation_tools/timing_observer.h>
#include <rtcus_navigation_tools/state_estimation_TF_publisher.h>
#include <rtcus_navigation_tools/obstacles_local_frame_publisher.h>
#include <rtcus_navigation_tools/simulation_motion_model_prediction_error_watch.h>
#include <rtcus_navigation/navigation_node_factory.h>

using namespace rtcus_shared_dwa;
using namespace rtcus_dwa;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation_node");
  RTCUS_ASSERT_CATCH_SIGSEGV();

  ROS_INFO("DWA Module. Creating shared dwa-navigation node...");

  DwaSharedLocalPlanner::TNavigationNodePtr nn;
  nn = rtcus_navigation::NavigationNodeFactory<DwaSharedLocalPlanner>::create_navigation_node();

  rtcus_navigation_tools::TFPublisher<DwaSharedLocalPlanner::TNavigationNode> tfpub(nn);
  rtcus_navigation_tools::LocalPointCloudObstaclesPublisher<DwaSharedLocalPlanner::TNavigationNode> lp(*nn);
  rtcus_navigation_tools::SimulatorPredictionMotionModelErrorWatch<DwaSharedLocalPlanner::TNavigationNode> spmmew(*nn);

  ROS_INFO("creating timing wrapper");
  rtcus_navigation_tools::TimeObserver<DwaSharedLocalPlanner::TNavigationNode> timing(nn);

  nn->start();
  ros::spin();
  return 0;
}

