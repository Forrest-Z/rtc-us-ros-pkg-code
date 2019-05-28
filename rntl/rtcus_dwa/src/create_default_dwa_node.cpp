/*
 * create_default_dwa_node.cpp
 *
 *  Created on: Jul 10, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_navigation/navigation_node.h>
#include <rtcus_navigation/impl/navigation_node.h>

//rtcus_navigation existing components
#include <rtcus_navigation/world_perception_ports/laser_scan.h>
#include <rtcus_navigation/world_perception_ports/uncertainty_obstacles_decorator.h>
#include <rtcus_navigation/world_perception_ports/point_cloud.h>

#include <rtcus_navigation/action_ports/adaptable_action_port.h>
#include<rtcus_navigation/action_ports/action_port_stamped_twist2d.h>

#include <rtcus_navigation/goal_ports/adaptable_conversion_goal_port.h>
#include <rtcus_navigation/goal_ports/waypoints_goal_port.h>
#include <rtcus_navigation/state_estimation/default_state_estimation.h>
#include <rtcus_navigation/state_estimation/non_time_correction_state_estimation.h>

#include <rtcus_navigation/state_ports/adaptable_conversion_state_port.h>
#include <rtcus_navigation/state_ports/high_noise_odometry_state_port.h>

#include <rtcus_navigation/goal_reach_detection/default_goal_reach.h>
#include <rtcus_navigation/shape_models/polygonal_configurable_shape_model.h>
#include <rtcus_navigation/kinodynamic_models/configurable_static_robot_kinodynamics.h>

//dwa component implementations
#include <rtcus_dwa/simple_dwa_default_planner.h>
#include <rtcus_dwa/dwa_motion_model.h>
#include <rtcus_navigation/action_ports/simulate_user_joystick_action_port.h>
#include <rtcus_navigation/collision_checkers/collision_cheker_polygonal_robot.h>

using namespace geometry_msgs;
using namespace nav_msgs;
using namespace boost;

using namespace rtcus_nav_msgs;
using namespace rtcus_motion_models;
using namespace rtcus_navigation;
using namespace rtcus_navigation::state_estimation;
using namespace rtcus_navigation::world_perception_ports;
using namespace rtcus_navigation::state_ports;
using namespace rtcus_navigation::action_ports;
using namespace rtcus_navigation::goal_ports;
using namespace rtcus_navigation::goal_reach_detection;
using namespace rtcus_navigation::shape_models;
using namespace rtcus_navigation::kinodynamic_models;

namespace rtcus_navigation
{
using namespace rtcus_dwa;

NavigationNodeFactory<DwaLocalPlanner>::TNavigationNodePtr NavigationNodeFactory<DwaLocalPlanner>::create_navigation_node()
{
  TNavigationNodePtr nn = create();
  std::string action_port_type;
  if (nn->getPrivateNode().getParam("action_port/output_type", action_port_type))
  {
    if (action_port_type == "Twist")
      nn->setActionPort(make_shared<ROSAdaptableActionPort<Twist2D, Twist> >());
    else if (action_port_type == "StampedTwist2D")
      nn->setActionPort(make_shared<ActionPortStampedTwist2D>());
    else
    {
      ROS_FATAL("Node Factory. Incorrect Action Port");
      exit(0);
    }
  }
  else
    nn->setActionPort(make_shared<ROSAdaptableActionPort<Twist2D, Twist> >());
  nn->init(true);
  ROS_INFO("navigation node initializated");
  return nn;
}

NavigationNodeFactory<DwaLocalPlanner>::TNavigationNodePtr NavigationNodeFactory<DwaLocalPlanner>::create()
{
  TNavigationNodePtr nn = make_shared<TNavigationNode>();

  ros::NodeHandle nh = nn->getPrivateNode();
  //------------------ STATE PORT--------------------------------------------------
  bool use_high_noise_state_port = false;
  if (!nh.getParam("factory/use_high_noise_state_port", use_high_noise_state_port))
    nh.setParam("factory/use_high_noise_state_port", use_high_noise_state_port);

  if (use_high_noise_state_port)
    nn->setStatePort(make_shared<HighNoiseOdometryStatePort>());
  else
    nn->setStatePort(make_shared<AdaptableConversionStatePort<DynamicState2D, Odometry> >());
  //-------------------------------------------------------------------------------

  ROS_INFO("Creating world perception port...");
  std::string input_type;
  if (!nh.getParam("perception_port/input_type", input_type) || input_type == "sensor_msgs/LaserScan")
  {
    nn->setWorldPerceptionPort(
        shared_ptr<UncertaintyObstaclesDecorator<pcl::PointXY> >(
            new UncertaintyObstaclesDecorator<pcl::PointXY>(
                make_shared<PointCloudObstaclesLaserScan<pcl::PointXY> >())));
  }
  else if (input_type == "sensor_msgs/PointCloud2")
  {
    nn->setWorldPerceptionPort(
        shared_ptr<UncertaintyObstaclesDecorator<pcl::PointXY> >(
            new UncertaintyObstaclesDecorator<pcl::PointXY>(make_shared<PointCloudObstacles<pcl::PointXY> >())));
  }

  //-------------------------------------------------------------------------------
  input_type="";
  ROS_INFO("Creating goal port...");
  if (!nh.getParam("goal_port/input_type", input_type) || input_type == "geometry_msgs/PoseStamped")
    nn->setGoalPort(make_shared<AdaptableConversionGoalPort<pcl::PointXY, PoseStamped> >());
  else if (input_type == "nav_msgs/Path")
    nn->setGoalPort(shared_ptr<WayPointsGoalPort>(new WayPointsGoalPort(nn)));
  else
  {
    ROS_FATAL("Wrong goal port input type %s", input_type.c_str());
    RTCUS_ASSERT(false);
  }

//set the DWA local planner
  shared_ptr<DwaLocalPlanner> dwa_planner = make_shared<DwaLocalPlanner>();
  //if (dwa_config != NULL)
  //{
  //  dwa_planner->updateConfig(*dwa_config);
  //}

  ROS_INFO("Creating motion planner...");
  nn->setNavigationPlanner(dwa_planner);
  //-------------------------------------------------------------------------------
  nn->setReachedGoalPredicate(make_shared<DefaultGoalReachPredicate>());
  //COVARIANT COLLISION CHECKER
  nn->setCovariantCollisionChecker(dwa_planner->collision_checker_);

  shared_ptr<PolygonalConfigurableRobotShapeModel> shape_model = make_shared<PolygonalConfigurableRobotShapeModel>();
  shape_model->setNavigationNode(*nn);
  nn->setRobotShapeModel(shape_model);
  nn->setKinodynamicModel(make_shared<ConfigurableKinodynamicModel<NonHolonomicKinoDynamicsConfig> >());

  return nn;
}

}

