/*
 * shared_control_acceleration_heading_cost_strategy.cpp
 *
 *  Created on: Nov 13, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_shared_dwa/heading_strategies/shared_control_acceleration_heading_strategy.h>

namespace rtcus_shared_dwa
{
namespace heading_strategies
{

SharedControlAccelerationHeadingCostStrategy::SharedControlAccelerationHeadingCostStrategy()
{
}

void SharedControlAccelerationHeadingCostStrategy::init(const DwaConfig& config, const Twist2D& state,
                                                        const DWAMotionModel& motion_model, const Twist2D& goal)
{
  //ROS_INFO(" *  Creating shared control strategy: ACCELERATION STRATEGY");
  SharedControlHeadingCostStrategy::init(config, state, motion_model, goal);
}

/*

 t_float SharedControlAccelerationHeadingCostStrategy::compute_MapU_distance(
 t_float v, t_float omega, const rtcus_nav_msgs::Twist2D& goal) const
 {
 rtcus_nav_msgs::Twist2D delta_goal;
 delta_goal.angular = (omega - goal.angular);
 delta_goal.linear = (v - goal.linear);

 return sqrt(pow(delta_goal.linear, 2) + pow(delta_goal.angular, 2)) / normalization;
 }
 */

t_float SharedControlAccelerationHeadingCostStrategy::computeCost(const DwaCommandCost& action)
{
  RTCUS_ASSERT_MSG(this->goal_->linear >= -1.0 && this->goal_->linear <= 1.0,
                   "inputs of the action should be normalized linear %lf angular %lf", action.linear, action.angular);
  RTCUS_ASSERT_MSG(this->goal_->angular >= -1.0 && this->goal_->angular <= 1.0,
                   "inputs of the action should be normalized linear %lf angular %lf", action.linear, action.angular);

  //NORMALIZE ACTION TO DYNAMIC LIMITS
  //v in [-1.0, 1.0], omega in [-1.0,1.0]
  t_float v_rel = action.linear / this->config_->get_max_linear_velocity();
  t_float omega_rel = action.angular / this->config_->get_max_angular_velocity();

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

