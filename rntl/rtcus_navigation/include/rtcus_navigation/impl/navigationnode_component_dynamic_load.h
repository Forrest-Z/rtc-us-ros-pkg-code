/*
 * navigationnode_component_dynamic_load.h
 *
 *  Created on: Dec 17, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef NAVIGATIONNODE_COMPONENT_DYNAMIC_LOAD_H_
#define NAVIGATIONNODE_COMPONENT_DYNAMIC_LOAD_H_

#include <rtcus_navigation/navigation_node.h>
#include <rtcus_navigation/task_status.h>
#include <pluginlib/class_loader.h>
#include <boost/type_traits.hpp>
#include <rtcus_assert/rtcus_assert.h>
#include <vector>
#include <boost/algorithm/string/predicate.hpp>

namespace rtcus_navigation
{
using namespace std;
using namespace rtcus_navigation;
using namespace boost;
using namespace rtcus_stamp;

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  bool NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::loadStateEstimation(
      const string& state_estimator_name)
  {

    ROS_INFO( "Trying to load dynamically a new state estimation component: %s. ", state_estimator_name.c_str());
    pluginlib::ClassLoader<NavigationNodeComponent> loader("rtcus_navigation",
                                                           "rtcus_navigation::NavigationNodeComponent");
    try
    {
      shared_ptr<NavigationNodeComponent> new_component = shared_ptr<NavigationNodeComponent>(
          loader.createUnmanagedInstance(state_estimator_name));

      shared_ptr<rtcus_navigation::StateEstimation<StateType, ActionType, TimeModel> > new_state_estimation =
          dynamic_pointer_cast<rtcus_navigation::StateEstimation<StateType, ActionType, TimeModel> >(new_component);

      if (!this->getStateEstimation() && new_state_estimation)
      {
        ROS_INFO("OK no state estimation still asigned.");
        this->setStateEstimation(new_state_estimation);
        ROS_INFO("Success.");
        return true;
      }
      else if (this->getStateEstimation() && new_state_estimation
          && typeid(*this->getStateEstimation()) != typeid(*new_state_estimation))
      {
        ROS_INFO(
            "Existing state estimation module type [%s]. New type [%s]", typeid(*this->getStateEstimation()).name(), typeid(*new_state_estimation).name());
        this->setStateEstimation(new_state_estimation);
        ROS_INFO("Success.");
        //new_state_estimation->register_meta_info();
        return true;
      }
      else
      {
        ROS_INFO("Loading canceled since there current type is the same it is being loaded.");
        return false;
      }
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("Load canceled. The plugin failed to load for some reason. Error: %s", ex.what());
      return false;
    }
  }
template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  bool NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::loadWorldPerceptionPort(
      const string& perception_port_name)
  {
    throw ros::Exception("do plugin lib");
  }

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  bool NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::loadGoalPort(
      const string& goal_port_name)
  {
    throw ros::Exception("do plugin lib");
  }

}

#include <rtcus_navigation/state_port.h>
#include <rtcus_navigation/state_ports/adaptable_conversion_state_port.h>
#include <rtcus_nav_msgs/StampedTwist2D.h>
#include <rtcus_nav_msgs/Twist2D.h>
#include <geometry_msgs/Twist.h>

namespace rtcus_navigation
{
using namespace rtcus_navigation::state_ports;
using namespace geometry_msgs;

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  bool NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::loadStatePort(
      const string& state_correction_port_name)
  {
    /*
     if (boost::starts_with(state_correction_port_name, "adapter_"))
     {
     if (boost::ends_with(state_correction_port_name, "_Twist2D"))
     {
     this->setStatePort(make_shared<AdaptableConversionStatePort<StateType, Twist2D> >());
     }
     else if (boost::ends_with(state_correction_port_name, "_StampedTwist2D"))
     {
     this->setStatePort(make_shared<AdaptableConversionStatePort<StateType, StampedTwist2D> >());
     }
     else if (boost::ends_with(state_correction_port_name, "_Twist"))
     {
     this->setStatePort(boost::make_shared<AdaptableConversionStatePort<StateType, Twist> >());
     }
     else
     {
     ROS_ERROR("Trying to load dynamically an invalid StatePort %s. Not state port assigned.", state_correction_port_name.c_str());
     }
     }
     */
    return false;
  }

}

#include <boost/mpl/if.hpp>
#include <rtcus_navigation/action_ports/adaptable_action_port.h>
#include<rtcus_navigation/action_ports/action_port_stamped_twist2d.h>
#include <geometry_msgs/Twist.h>

namespace rtcus_navigation
{

using namespace rtcus_navigation::action_ports;
template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  bool NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::loadActionPort(
      const std::string& action_port)
  {
    /*
     ROS_INFO( "Action Space Dynamic Load [%s].Available options: twist, twist2d_stamped", action_port.c_str());

     if (action_port == "twist")
     {
     shared_ptr<TActionPort> ap = dynamic_pointer_cast<TActionPort>(make_shared<rtcus_navigation::Act>());
     if (ap)
     this->setActionPort(ap);
     else
     ROS_ERROR(
     "Incorrect action port loading: immposible convert from %s to %s", typeid(ActionPortStampedTwist2D).name(), typeid(TActionPort).name());

     }
     else if (action_port == "twist_2d_stamped")
     {
     shared_ptr<TActionPort> ap = dynamic_pointer_cast<TActionPort>(make_shared<ActionPortStampedTwist2D>());
     if (ap)
     this->setActionPort(ap);
     else
     ROS_ERROR(
     "Incorrect action port loading: immposible convert from %s to %s", typeid(ActionPortStampedTwist2D).name(), typeid(TActionPort).name());
     }
     else
     {
     ROS_ERROR("Incorrect use of the action_library");
     throw ros::Exception("do plugin lib");
     }
     */
    return false;
  }

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  bool NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::loadNavigationPlanner(
      const string& navigation_planner_name)
  {
    throw ros::Exception("do plugin lib");
  }
template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  bool NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::loadReachedGoalPredicate(
      const string& reach_goal_predicate_name)
  {
    throw ros::Exception("do plugin lib");
  }

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  bool NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::loadKinodynamicModel(
      const std::string& name)
  {
    throw ros::Exception("do plugin lib");
  }

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  bool NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::loadRobotShapeModel(
      const string& name)
  {

    throw ros::Exception("do plugin lib");
  }

}

#endif /* NAVIGATIONNODE_COMPONENT_DYNAMIC_LOAD_H_ */
