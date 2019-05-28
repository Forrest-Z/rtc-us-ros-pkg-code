/*
 * dwa_demo_test.cpp
 *
 *  Created on: Jul 11, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <rtcus_assert/rtcus_assert.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "navigation_node");
  RTCUS_ASSERT_CATCH_SIGSEGV();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

#include <rtcus_navigation/navigation_node.h>
#include <rtcus_navigation/impl/navigation_node.h>
#include <rtcus_dwa/simple_dwa_default_planner.h>
#include <rtcus_dwa/dwa_motion_model.h>

#include <rtcus_navigation_tools/timing_observer.h>
#include <rtcus_navigation_tools/state_estimation_TF_publisher.h>
#include <rtcus_navigation_tools/obstacles_local_frame_publisher.h>
#include <rtcus_navigation_tools/simulation_motion_model_prediction_error_watch.h>
#include <rtcus_dwa/visual_representation/dwa_algorithm_representation.h>
#include <rtcus_navigation/navigation_node_factory.h>

using namespace rtcus_dwa;
using namespace boost;

TEST(dwa_demo_test, basic_test1)
{
  ros::NodeHandle nh;
  //wait for some real state information
  ros::spinOnce();
  ros::Duration(0.01).sleep();
  ros::spinOnce();

  ROS_INFO("creating dwa navigation node...");

  DwaLocalPlanner::TNavigationNodePtr nn =
      rtcus_navigation::NavigationNodeFactory<DwaLocalPlanner>::create_navigation_node();

  ROS_INFO("creating funcionality extensions for testing...");
  rtcus_navigation_tools::TimeObserver<DwaLocalPlanner::TNavigationNode> timing(nn);
  rtcus_navigation_tools::TFPublisher<DwaLocalPlanner::TNavigationNode> tfpub(nn);
  rtcus_navigation_tools::LocalPointCloudObstaclesPublisher<DwaLocalPlanner::TNavigationNode> lp(*nn);
  rtcus_navigation_tools::SimulatorPredictionMotionModelErrorWatch<DwaLocalPlanner::TNavigationNode> spmmew(*nn);

  ROS_INFO("setting new artificial goal extensions for testing...");
  StampedData<PointXY> goal;
  goal.getData().x = 15;
  goal.getData().y = 0;
  goal.setStamp(ros::Time(0));
  goal.setFrameId(nn->getReferenceFrame());
  nn->getGoalPort()->pushGoal(goal);
  ROS_INFO("Node Starting for testing..");

  nn->start();
  ros::spin();
}

