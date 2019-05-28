/*
 *  Created on: April 13, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */
#include <rtcus_navigation/navigation_node.h>
#include <rtcus_navigation/impl/navigation_node.h>
#include <rtcus_dwa/simple_dwa_default_planner.h>
#include <rtcus_dwa/dwa_motion_model.h>
#include <rtcus_assert/rtcus_assert.h>

#include <rtcus_navigation_tools/timing_observer.h>
#include <rtcus_navigation_tools/state_estimation_TF_publisher.h>
#include <rtcus_navigation_tools/obstacles_local_frame_publisher.h>
#include <rtcus_navigation_tools/simulation_motion_model_prediction_error_watch.h>
#include <rtcus_dwa/visual_representation/dwa_algorithm_representation.h>
#include <rtcus_navigation/navigation_node_factory.h>
#include <rtcus_navigation/action_ports/simulate_user_joystick_action_port.h>

using namespace rtcus_dwa;

namespace rtcus_navigation
{
NavigationNodeFactory<DwaLocalPlanner>::TNavigationNodePtr NavigationNodeFactory<DwaLocalPlanner>::create_navigation_node(
    const std::string& type)
{
  TNavigationNodePtr nn = NavigationNodeFactory<DwaLocalPlanner>::create();
  if (type == "simulated_user")
  {
    shared_ptr<SimulateUserJoystickActionPort> action_port = boost::shared_ptr<SimulateUserJoystickActionPort>(
        new SimulateUserJoystickActionPort(nn->getKinodynamicModel()));
    nn->setActionPort(action_port);
  }
  nn->init(true);
  return nn;
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simulated_user");
  ROS_INFO("Creating simulated user node...");
  DwaLocalPlanner::TNavigationNodePtr nn =
      rtcus_navigation::NavigationNodeFactory<DwaLocalPlanner>::create_navigation_node("simulated_user");

  rtcus_navigation_tools::TimeObserver<DwaLocalPlanner::TNavigationNode> timing(nn);
  rtcus_navigation_tools::TFPublisher<DwaLocalPlanner::TNavigationNode> tfpub(nn);
  rtcus_navigation_tools::LocalPointCloudObstaclesPublisher<DwaLocalPlanner::TNavigationNode> lp(*nn);
  rtcus_navigation_tools::SimulatorPredictionMotionModelErrorWatch<DwaLocalPlanner::TNavigationNode> spmmew(*nn);

  nn->start();
  ros::spin();
}

