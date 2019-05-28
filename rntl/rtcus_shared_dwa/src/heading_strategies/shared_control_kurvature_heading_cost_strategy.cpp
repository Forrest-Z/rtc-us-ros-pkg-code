/*
 * shared_control_kurvature_heading_cost_strategy.cpp
 *
 *  Created on: Nov 13, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_shared_dwa/heading_strategies/shared_control_kurvature_strategy.h>

namespace rtcus_shared_dwa
{
namespace heading_strategies
{

SharedControlKurvatureHeadingCostStrategy::SharedControlKurvatureHeadingCostStrategy()
{
}

void SharedControlKurvatureHeadingCostStrategy::init(const DwaConfig& config, const Twist2D& state,
                                                     const DWAMotionModel& motion_model,
                                                     const rtcus_nav_msgs::Twist2D& goal)
{
  SharedControlHeadingCostStrategy::init(config, state, motion_model, goal);

  //TODO: Understand better this, take alpha from normalized or from the original?
  //normalized is better to explore uniformly the action capabilities of the vehicle
  //non normaliezed, standarizes the joystick order independently to the vehicle
  //remarks, in any case, if normalized, then take also the action normalized..
  if (normalized_goal_.linear == 0 && normalized_goal_.angular == 0)
  {
    this->goal_alpha_ = std::numeric_limits < t_float > ::quiet_NaN();
  }
  else
  {
    this->goal_alpha_ = atan2(this->goal_->linear, this->goal_->angular);

    if (this->goal_alpha_ < 0.0)
    {
      //Somtimes we get -0.0 and this exploit. It doesn't mean backwards, and in fact it is not allowed this case.
      this->goal_alpha_ += 2.0 * M_PI;
    }
    this->goal_magnitude_ = sqrt(
        normalized_goal_.angular * normalized_goal_.angular + normalized_goal_.linear * normalized_goal_.linear)
        / sqrt(2.0);

    RTCUS_ASSERT_MSG(this->goal_->linear >= 0, "goal alpha: %lf, goal linear: %lf, goal angular: %lf",
                     this->goal_alpha_, this->goal_->linear, this->goal_->angular);
  }
}

t_float SharedControlKurvatureHeadingCostStrategy::computeCost(const rtcus_dwa::CommandCost<rtcus_nav_msgs::Twist2D>& action)
{
  t_float total_cost;
  if (normalized_goal_.linear == 0)
  {
    //DEGENERATED CASE GOAL KURVATURE NOT DEFINED
    //POTENTIAL FIELD TO THE NULL ACTION... AVOID ROTATING ON ITSELF??
    total_cost = fabs(normalized_goal_.angular - action.getAction().angular / this->config_->get_max_angular_velocity()) / 2.0;

    RTCUS_ASSERT_MSG(total_cost >= 0.0 && total_cost <= 1.0,
                     "Degenerated case: goal (linear: %lf, angular: %lf) ->  final total heading cost: %lf",
                     normalized_goal_.linear, normalized_goal_.angular, total_cost);
  }
  else
  {
    t_float kurvature_alpha= ((const DwaCommandCost&) action).getKurvatureAlpha();
    t_float kurvature_relative_error = fabs(this->goal_alpha_ - kurvature_alpha) / M_PI;
    ROS_DEBUG(
        " * kurvature error %lf, goal alpha %lf, action alpha %lf", kurvature_relative_error, goal_alpha_, kurvature_alpha);
    RTCUS_ASSERT_MSG(kurvature_relative_error >= 0.0 && kurvature_relative_error <= 1.0,
                     " * kurvature error %lf, goal alpha %lf, action alpha %lf", kurvature_relative_error, goal_alpha_,
                     kurvature_alpha);

    total_cost = kurvature_relative_error;
    RTCUS_ASSERT_MSG(total_cost >= 0.0 && total_cost <= 1.0, "total heading cost-> %lf", total_cost);
  }

  this->normalize(total_cost);
  return total_cost;
}
/*
 * THIS ALGORITHM BELOW HAS SHOWNED TO WORK PERFECTLY BUT IT HAS AN INTEGRATED APPROACH OF HEADING AND VELOCITY: TARGET-> SPLIT THEM
 * {
 RTCUS_ASSERT_MSG(action.linear >= 0 && goal_->linear >= 0, "This heading method is not prepared for backwards speed");
 //so both alphas belong to [0,PI]

 //The goal action should be provided normalized to the maxium kinodynamic capabilities
 t_float velocity_error = fabs(
 this->normalized_goal_.linear - action.linear / this->config_->getKinodynamicConfig().linear_forward_speed_limit);

 ROS_DEBUG(
 " * velocity_error error %lf, goal linear %lf, action linear %lf", velocity_error, normalized_goal_.linear, action.linear);
 RTCUS_ASSERT_MSG(velocity_error >= 0.0 && velocity_error <= 1.0, "action linear %lf, goal linear %lf -> error %lf",
 action.linear, this->normalized_goal_.linear, velocity_error);

 t_float total_cost;
 //DEGENERATED CASES
 if (normalized_goal_.angular == 0 && normalized_goal_.linear == 0)
 {
 //potential field to the stop action in the action space
 total_cost = velocity_error * 0.5 + 0.5 * fabs(action.angular) / this->config_->get_max_angular_velocity();
 }
 //GOAL KURVATURE NOT DEFINED
 else if (normalized_goal_.linear == 0)
 {
 //AVOID ROTATING ON ITSELF
 total_cost = fabs(normalized_goal_.angular - action.angular / this->config_->get_max_angular_velocity()) / 2.0;

 RTCUS_ASSERT_MSG(
 total_cost >= 0.0 && total_cost <= 1.0,
 "Degenerated case: goal (linear: %lf, angular: %lf) -> pre-velocity-error: %lf, final total cost: %lf",
 normalized_goal_.linear, normalized_goal_.angular, velocity_error, total_cost);
 }
 else
 {
 t_float kurvature_relative_error = fabs(this->goal_alpha_ - action.getKurvatureAlpha()) / M_PI;
 ROS_DEBUG(
 " * kurvature error %lf, goal alpha %lf, action alpha %lf", kurvature_relative_error, goal_alpha_, action.getKurvatureAlpha());
 RTCUS_ASSERT_MSG(kurvature_relative_error >= 0.0 && kurvature_relative_error <= 1.0,
 " * kurvature error %lf, goal alpha %lf, action alpha %lf", kurvature_relative_error, goal_alpha_,
 action.getKurvatureAlpha());

 total_cost = this->shared_contro_config_.k_shared_maintain_kuvature * kurvature_relative_error
 + (1.0 - this->shared_contro_config_.k_shared_maintain_kuvature) * velocity_error;
 ROS_DEBUG(
 "total cost (kurvature error ponderation: %lf) -> %lf", this->shared_contro_config_.k_shared_maintain_kuvature, total_cost);
 RTCUS_ASSERT_MSG(total_cost >= 0.0 && total_cost <= 1.0, "total cost (kurvature error ponderation: %lf) -> %lf",
 this->shared_contro_config_.k_shared_maintain_kuvature, total_cost);

 }

 this->normalize(total_cost);
 return total_cost;
 }
 */
}

}

