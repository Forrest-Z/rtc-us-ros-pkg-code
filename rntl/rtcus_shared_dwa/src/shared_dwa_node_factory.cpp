/*
 * shared_dwa_node_factory.cpp
 *
 *  Created on: Dec 5, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_shared_dwa/shared_dwa_navigation_planner.h>
#include <rtcus_navigation/impl/navigation_node.h>

//SELECTING NAVIGATION ARCHITECTURE BUILDING BLOCKS
#include <rtcus_navigation/goal_reach_detection/no_goal_reach_detection.h>
#include <rtcus_navigation/goal_ports/shared_control_proportial_dynamics_goal_port.h>
#include <rtcus_navigation/action_ports/adaptable_action_port.h>
#include <rtcus_navigation/action_ports/action_port_stamped_twist2d.h>

#include <rtcus_navigation/state_estimation/non_time_correction_state_estimation.h>
#include <rtcus_navigation/shape_models/polygonal_configurable_shape_model.h>
#include <rtcus_navigation/kinodynamic_models/configurable_static_robot_kinodynamics.h>

#include <rtcus_navigation/state_ports/adaptable_conversion_state_port.h>
#include <rtcus_navigation/state_ports/high_noise_odometry_state_port.h>

#include <rtcus_navigation/world_perception_ports/uncertainty_obstacles_decorator.h>
#include <rtcus_navigation/world_perception_ports/laser_scan.h>
#include <geometry_msgs/Twist.h>
#include <rtcus_navigation/navigation_node_factory.h>

using namespace rtcus_navigation;
using namespace rtcus_navigation::goal_ports;
using namespace rtcus_navigation::state_ports;
using namespace rtcus_navigation::action_ports;
using namespace rtcus_navigation::world_perception_ports;
using namespace rtcus_navigation::state_estimation;
using namespace rtcus_navigation::shape_models;
using namespace rtcus_navigation::kinodynamic_models;
using namespace geometry_msgs;

namespace rtcus_navigation
{
using namespace rtcus_dwa;
using namespace rtcus_shared_dwa;
template<>
  class NavigationNodeFactory<DwaSharedLocalPlanner>
  {
  public:
    NAVIGATION_TYPES (DwaSharedLocalPlanner);

  protected:
    static TNavigationNodePtr create();

  public:
    static TNavigationNodePtr create_navigation_node();

    //specific construction
    static TNavigationNodePtr create_navigation_node(const std::string& type);

  };

NavigationNodeFactory<DwaSharedLocalPlanner>::TNavigationNodePtr NavigationNodeFactory<DwaSharedLocalPlanner>::create()
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

  nn->setWorldPerceptionPort(
      shared_ptr<UncertaintyObstaclesDecorator<pcl::PointXY> >(
          new UncertaintyObstaclesDecorator<pcl::PointXY>(make_shared<PointCloudObstaclesLaserScan<pcl::PointXY> >())));

//nn->setStateEstimation(make_shared<NonTimeCorrectionStateEstimation<DynamicState2D, Twist2D> >());

//set the DWA local planner
  shared_ptr<DwaSharedLocalPlanner> dwa_planner = make_shared<DwaSharedLocalPlanner>();

  nn->setKinodynamicModel(make_shared<ConfigurableKinodynamicModel<NonHolonomicKinoDynamicsConfig> >());

  nn->setGoalPort(
      shared_ptr<SharedControlProportionalDynamicsGoalPort>(new SharedControlProportionalDynamicsGoalPort(*nn)));

  nn->setNavigationPlanner(dwa_planner);

  nn->setReachedGoalPredicate(
      make_shared<rtcus_navigation::goal_reach_detection::NoReachedGoalPredicate<DynamicState2D, Twist2D> >());

  nn->setCovariantCollisionChecker(dwa_planner->collision_checker_);

  shared_ptr<PolygonalConfigurableRobotShapeModel> shape_model = make_shared<PolygonalConfigurableRobotShapeModel>();
  shape_model->setNavigationNode(*nn);
  nn->setRobotShapeModel(shape_model);

  return nn;
}
//default construction

NavigationNodeFactory<DwaSharedLocalPlanner>::TNavigationNodePtr NavigationNodeFactory<DwaSharedLocalPlanner>::create_navigation_node()
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
  {
    nn->setActionPort(make_shared<ROSAdaptableActionPort<Twist2D, Twist> >());
  }

  nn->init(true);
  ROS_INFO("navigation node initializated");
  return nn;

}

}
//---------------------------------------------------------------------------------------------------
#include <rtcus_compositions/state_composer.h>
namespace rtcus_compositions
{
using namespace rtcus_nav_msgs;

/*reinterpret the action goal like itself.*/
template<>
  void StateComposer::inverse_compose(const Twist2D& target, const DynamicState2D& local_frame_state, Twist2D& dst,
                                      const std::string& new_frame_name)
  {
    dst = target;
  }
}
