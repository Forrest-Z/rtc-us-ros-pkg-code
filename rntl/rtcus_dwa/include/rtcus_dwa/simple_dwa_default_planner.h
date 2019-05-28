/*
 * simple_dwa_default_planner.h
 *
 *  Created on: Dec 7, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef SIMPLE_DWA_DEFAULT_PLANNER_H_
#define SIMPLE_DWA_DEFAULT_PLANNER_H_

#include <rtcus_dwa/simple_dwa_ros.h>
//DWA Cost Computation Strategies
#include <rtcus_dwa/heading_strategies/trajectory_goal_intersection_heading_strategy.h>
#include <rtcus_dwa/heading_strategies/default_dwa_heading_cost_strategy.h>
#include <rtcus_dwa/velocity_strategies/default_velocity_strategy.h>

namespace rtcus_dwa
{
//TODO: Rename this header file coherently with the classname
class DwaLocalPlanner : public DwaLocalPlannerBase<PointXY>
{
public:
  typedef pcl::PointCloud<pcl::PointXY> PointCloudXY;
  //TODO: Replace this for a traits class
  DECLARE_NAVIGATION_PLANNER_TYPES_EXPLICIT(rtcus_nav_msgs::DynamicState2D, PointCloudXY,rtcus_nav_msgs::Twist2D,PointXY,PolygonalRobot,NonHolonomicKinoDynamicsConfig,rtcus_navigation::ROSTimeModel);

  virtual void init(TNavigationNode& host_node)
  {
    DwaLocalPlannerBase<PointXY>::init(host_node);
    //-----VELOCITY STRATEGY ---
    this->velocity_strategy_ = make_shared<velocity_cost_strategies::DefaultVelocityStrategy<PointXY> >();

    //---- HEADING COST STRATEGY---
    std::string heading_cost_strategy;
    if (this->component_node_.getParam("heading_cost_strategy", heading_cost_strategy))
    {
      if (heading_cost_strategy == "trajectory_goal_intersection")
      {
        this->heading_cost_strategy_ = make_shared<
            rtcus_dwa::heading_strategies::TragectoryGoalIntersectionHeadingCostStrategy>();
        this->component_node_.setParam("heading_cost_strategy", "trajectory_goal_intersection");
      }
      else if (heading_cost_strategy == "face_goal_dwa_default")
      {
        this->heading_cost_strategy_ = make_shared<rtcus_dwa::heading_strategies::DefaultDwaHeadingCostStrategy>();
        this->component_node_.setParam("heading_cost_strategy", "face_goal_dwa_default");
      }
      else
      {
        ROS_FATAL("Incorrect heading cost strategy: %s. The application will end.", heading_cost_strategy.c_str());
        exit(0);
      }
    }
    else
    {
      ROS_INFO("Selecting default Heading Cost strategy: trajectory_goal_intersection");
      this->component_node_.setParam("heading_cost_strategy", "trajectory_goal_intersection");
      this->heading_cost_strategy_ = make_shared<
          rtcus_dwa::heading_strategies::TragectoryGoalIntersectionHeadingCostStrategy>();
    }
  }

  virtual ~DwaLocalPlanner()
  {

  }
};
}

#include <rtcus_navigation/navigation_node_factory.h>
namespace rtcus_navigation
{
using namespace rtcus_dwa;

template<>
  class NavigationNodeFactory<DwaLocalPlanner>
  {
  public:
    typedef DwaLocalPlanner::TNavigationNodePtr TNavigationNodePtr;
    typedef DwaLocalPlanner::TNavigationNode TNavigationNode;

  protected:
    static TNavigationNodePtr create();

  public:

    //default construction
    static TNavigationNodePtr create_navigation_node();

    //specific construction
    static TNavigationNodePtr create_navigation_node(const std::string& type);

  };
}

#endif /* SIMPLE_DWA_DEFAULT_PLANNER_H_ */
