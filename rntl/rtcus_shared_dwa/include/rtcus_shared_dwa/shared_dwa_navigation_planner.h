/*
 * shared_dwa_navigation_planer.h
 *
 *  Created on: Dec 5, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef SHARED_DWA_NAVIGATION_PLANER_H_
#define SHARED_DWA_NAVIGATION_PLANER_H_

#include <rtcus_dwa/simple_dwa_ros.h>
#include <rtcus_shared_dwa/heading_strategies/shared_control_heading_cost_strategy.h>
#include <rtcus_shared_dwa/heading_strategies/shared_control_kurvature_strategy.h>
#include <rtcus_shared_dwa/heading_strategies/shared_control_acceleration_heading_strategy.h>
#include <rtcus_shared_dwa/velocity_strategies/shared_control_velocity_cost_strategy.h>
#include <dynamic_reconfigure/server.h>
#include <rtcus_shared_dwa/SharedControlConfig.h>

namespace rtcus_shared_dwa
{
using namespace rtcus_dwa;

class DwaSharedLocalPlanner : public DwaLocalPlannerBase<Twist2D>
{
private:
  shared_ptr<dynamic_reconfigure::Server<SharedControlConfig> > configure_server_;
  SharedControlConfig shared_contro_config_;

  const SharedControlConfig& update_shared_planner_params_cb(SharedControlConfig &config, uint32_t level)
  {
    return this->shared_contro_config_ = config;
  }

public:
  typedef pcl::PointCloud<pcl::PointXY> PointCloudXY;

  DECLARE_NAVIGATION_PLANNER_TYPES_EXPLICIT(rtcus_nav_msgs::DynamicState2D, PointCloudXY,rtcus_nav_msgs::Twist2D,rtcus_nav_msgs::Twist2D,PolygonalRobot,NonHolonomicKinoDynamicsConfig,rtcus_navigation::ROSTimeModel);

  virtual void reset()
  {
    DwaLocalPlannerBase<Twist2D>::reset();
  }

  virtual void init(TNavigationNode& navigation_node)
  {
    DwaLocalPlannerBase<Twist2D>::init(navigation_node);

    this->configure_server_ = shared_ptr<dynamic_reconfigure::Server<SharedControlConfig> >(
        new dynamic_reconfigure::Server<SharedControlConfig>(
            ros::NodeHandle(this->component_node_.getNamespace() + "/shared_dwa")));

    dynamic_reconfigure::Server<SharedControlConfig>::CallbackType f;
    f = boost::bind(&DwaSharedLocalPlanner::update_shared_planner_params_cb, this, _1, _2);
    this->configure_server_->setCallback(f);

    /*this->heading_cost_strategy = make_shared<rtcus_dwa::heading_strategies::SharedControlHeadingCostStrategy>();*/
    this->heading_cost_strategy_ = boost::make_shared<
        rtcus_shared_dwa::heading_strategies::SharedControlKurvatureHeadingCostStrategy>();

    this->velocity_strategy_ = make_shared<velocity_cost_strategies::SharedControlVelocityStrategy>();

    ROS_INFO(
        "* Shared DWA planner - using the heading cost strategy '%s'", getClassName(*this->heading_cost_strategy_).c_str());
  }

  virtual bool computeVelocityCommands(const pcl::PointCloud<pcl::PointXY>& obstacles, const Twist2D& goal,
                                       const DynamicState2D& local_state, Twist2D& resulting_action)
  {
    if (goal.linear >= 0)
    {
      return DwaLocalPlannerBase<Twist2D>::computeVelocityCommands(obstacles, goal, local_state, resulting_action);
    }
    else //BACWARD DIRECT CONTROL
    {
      resulting_action.linear = std::min(
          local_state.twist.linear + this->config_.getKinodynamicConfig().linear_brake_limit,
          std::max(goal.linear,
                   local_state.twist.linear - this->config_.getKinodynamicConfig().linear_backwards_speed_limit));

      resulting_action.angular = std::min(
          local_state.twist.angular + this->config_.getKinodynamicConfig().angular_acceleration_limit,
          std::max(goal.angular,
                   local_state.twist.angular - this->config_.getKinodynamicConfig().angular_acceleration_limit));

      return false;
    }

  }

  virtual ~DwaSharedLocalPlanner()
  {

  }
};
}

#endif /* SHARED_DWA_NAVIGATION_PLANER_H_ */
