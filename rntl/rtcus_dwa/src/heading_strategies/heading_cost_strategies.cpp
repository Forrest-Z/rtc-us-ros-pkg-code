/*
 * heading_cost_strategies.cpp
 *
 *  Created on: Aug 28, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_dwa/heading_cost_strategies.h>
#include <rtcus_dwa/heading_strategies/default_dwa_heading_cost_strategy.h>
#include <rtcus_dwa/common.h>

namespace rtcus_dwa
{
namespace heading_strategies
{
double calculateDifferenceBetweenAngles(double v, double omega, double first, double second)
{
  while (first > TWO_PI)
    first -= TWO_PI;
  while (first < 0)
    first += TWO_PI;
  while (second > TWO_PI)
    second -= TWO_PI;
  while (second < 0)
    second += TWO_PI;

  double a = max(first, second);
  double b = min(first, second);
  double diff = a - b;
  if (diff > M_PI)
    diff = TWO_PI - diff;
  //ROS_INFO("[v %lf omega %lf]first %lf second %lf a %lf b %lf -> diff %lf", v, omega, first, second, a, b, diff);
  return diff;

}

t_float DefaultDwaHeadingCostStrategy::computeCost(const CommandCost<Twist2D>& action)

{
  //ROS_INFO("predicting next state for heading");
  rtcus_nav_msgs::DynamicState2D predicted_state;
  this->motion_model_->predictLocalStateTransition(action.getAction(),
                                                   ros::Duration(this->config_->simulation_time_step), predicted_state);

  t_float goal_angle = atan2(this->goal_->y - predicted_state.pose.y, this->goal_->x - predicted_state.pose.x);
  t_float dist = calculateDifferenceBetweenAngles(action.getAction().linear, action.getAction().angular,
                                                  predicted_state.pose.phi, goal_angle);

  this->normalize(dist);
  return dist;
}
}
}

