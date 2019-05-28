/*
 * shared_control_heading_cost_strategy.cpp
 *
 *  Created on: Nov 2, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_shared_dwa/heading_strategies/shared_control_heading_cost_strategy.h>
#include <rtcus_shared_dwa/heading_strategies/shared_control_acceleration_heading_strategy.h>

namespace rtcus_shared_dwa
{
namespace heading_strategies
{
using namespace rtcus_dwa;

SharedControlHeadingCostStrategy::SharedControlHeadingCostStrategy()
{

}

SharedControlHeadingCostStrategy::~SharedControlHeadingCostStrategy()
{

}

void SharedControlHeadingCostStrategy::init(const DwaConfig& config, const Twist2D& state,
                                            const DWAMotionModel& motion_model, const rtcus_nav_msgs::Twist2D& goal)
{
  ROS_DEBUG(" Heading cost strategy, goal: %lf, %lf", goal.linear, goal.angular);
  HeadingCostStrategy<rtcus_nav_msgs::Twist2D, rtcus_nav_msgs::Twist2D,rtcus_nav_msgs::Twist2D>::init(config, state,
                                                                                              motion_model, goal);

  this->normalized_goal_.linear = this->goal_->linear
      / this->config_->getKinodynamicConfig().linear_forward_speed_limit;
  this->normalized_goal_.angular = goal_->angular / this->config_->getKinodynamicConfig().angular_speed_limit;

  if (this->normalized_goal_.linear < -0 || this->normalized_goal_.linear > 1.0 || this->normalized_goal_.angular < -0
      || this->normalized_goal_.angular > 1.0)
    ROS_WARN_STREAM_THROTTLE(
        1.0,
        " * " << getClassName(*this) << ", The given command trajectory goal is not kinematically possible for this robot: " << goal);

  this->normalized_goal_.linear = std::max(0.0, std::min(1.0, this->goal_->linear));
  this->normalized_goal_.angular = std::max(-1.0, std::min(1.0, this->goal_->angular));

}

t_float SharedControlHeadingCostStrategy::computeCost(const rtcus_dwa::CommandCost<rtcus_nav_msgs::Twist2D>& action)
{
  //NORMALIZE ACTION TO DYNAMIC LIMITS
  //v in [-1.0, 1.0], omega in [-1.0,1.0]
  t_float v_rel = action.getAction().linear / this->config_->getKinodynamicConfig().linear_forward_speed_limit;
  t_float omega_rel = action.getAction().angular / this->config_->getKinodynamicConfig().angular_speed_limit;

  Twist2D delta_twist;
  //take the error regarding the goal that is bounded in e_v=2.0 y e_omega=2.0
  delta_twist.linear = this->normalized_goal_.linear - v_rel;
  delta_twist.angular = this->normalized_goal_.angular - omega_rel;

  //ROS_INFO("Delta Twist %lf %lf -> %lf %lf", action.linear, action.angular, delta_twist.linear, delta_twist.angular);
  //Then. We get The euclidean error and normalize it.
  t_float res = min(1.0, sqrt(pow(delta_twist.linear, 2) + pow(delta_twist.angular, 2)) / sqrt(4.0));
  return res;
}

}

}

