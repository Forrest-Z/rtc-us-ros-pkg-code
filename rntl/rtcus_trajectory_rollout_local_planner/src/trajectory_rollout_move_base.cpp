/*
 * trajectory_rollout_move_base.cpp
 *
 *  Created on: Nov 23, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_trajectory_rollout_local_planner/trajectory_rollout_ros_local_planner.h>
#include <rtcus_navigation/navigation_node_factory.h>
#include <rtcus_navigation/navigation_node.h>
#include <rtcus_assert/rtcus_assert.h>
#include <rtcus_navigation_tools/timing_observer.h>
#include <rtcus_navigation_tools/obstacles_local_frame_publisher.h>
#include <rtcus_navigation_tools/simulation_motion_model_prediction_error_watch.h>

using namespace rtcus_navigation;

int main(int argc, char** argv)
{
  typedef NavigationNodeFactory<TrajectoryRolloutAlgorithmROS>::TNavigationNode TNavigationNode;
  typedef NavigationNodeFactory<TrajectoryRolloutAlgorithmROS>::TNavigationNodePtr TNavigationNodePtr;

  ros::init(argc, argv, "rtcus_dwa_local_planner");
  RTCUS_ASSERT_CATCH_SIGSEGV();

  ROS_INFO("DWA Module. Creating shared dwa-navigation node...");

  TNavigationNodePtr nn = NavigationNodeFactory<TrajectoryRolloutAlgorithmROS>::create_navigation_node();

  rtcus_navigation_tools::SimulatorPredictionMotionModelErrorWatch<TNavigationNode> spmmew(*nn);

  ROS_INFO("creating timing wrapper");
  rtcus_navigation_tools::TimeObserver<TNavigationNode> timing(nn);

  nn->start();
  ros::spin();
  return 0;
}
