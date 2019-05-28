/*
 * NavigationComponent.h
 *
 *  Created on: Jul 4, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef NAVIGATIONCOMPONENT_H_
#define NAVIGATIONCOMPONENT_H_

#include <boost/signal.hpp>
#include <boost/signals.hpp>
#include <rtcus_stamp/stamped.h>
#include <ros/ros.h>
#include <string>
#include <std_srvs/Empty.h>
#include <boost/units/detail/utility.hpp>

namespace rtcus_navigation
{
//---------------------------------------------------------------------------------------------
enum NavComponentType
{
  tNavigationNode, tNavigationPlanner, tActionPort, tStatePort, tStateEstimation, tGoalPort, tPerceptionPort,
  tCollisionChecker, tGoalReachedDetection, tShapeModel, tKinoDynamicModel
};
//---------------------------------------------------------------------------------------------
class NavigationNodeComponent
{
public:

  virtual NavComponentType getComponentType() const;
  virtual ros::NodeHandle& getComponentNode();
  virtual ros::NodeHandle& getPrivateNode();

  virtual std::string type_name() const;
  virtual std::string getArchitectureComponentName() const;

  void register_meta_info();
  virtual void reset()=0;
  virtual void unload();
  virtual ~NavigationNodeComponent();

protected:
  NavigationNodeComponent(NavigationNodeComponent& covariant_component);
  NavigationNodeComponent(NavComponentType type, std::string root_namespace = "~");

  NavComponentType type_;
  ros::NodeHandle node_;
  ros::NodeHandle component_node_;
  ros::NodeHandle private_node_;
  ros::ServiceServer reset_service_;
  bool registered_;
  bool reset_cb();
};

//---------------------------------------------------------------------------------------------
/* \brief The names of the different component types. These names are also used for the parameter server hierarchy*/
const static std::string __navComponentsNames[] = {std::string("navigation_node"), std::string("planner"), std::string(
    "action_port"),
                                                   std::string("state_port"), std::string("state_estimation"),
                                                   std::string("goal_port"), std::string("perception_port"),
                                                   std::string("collision_checker"), std::string(
                                                       "goal_reach_detection"),
                                                   std::string("shape_model"), std::string("kinodynamic_model")};

const std::string& getComponentTypeName(NavComponentType type);

}
#endif /* NAVIGATIONCOMPONENT_H_ */
