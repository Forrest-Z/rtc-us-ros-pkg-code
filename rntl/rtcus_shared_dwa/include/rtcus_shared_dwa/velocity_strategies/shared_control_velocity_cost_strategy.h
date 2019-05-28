/*
 * shared_control_velocity_cost_strategy.h
 *
 *  Created on: Feb 13, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef SHARED_CONTROL_VELOCITY_COST_STRATEGY_H_
#define SHARED_CONTROL_VELOCITY_COST_STRATEGY_H_

#include <rtcus_dwa/velocity_strategies/velocity_strategy.h>
#include <rtcus_nav_msgs/Twist2D.h>
namespace rtcus_shared_dwa
{
namespace velocity_cost_strategies
{
using namespace rtcus_dwa;

/*\brief Tries to maximize the velocity*/
class SharedControlVelocityStrategy : public VelocityStrategy<rtcus_nav_msgs::Twist2D, rtcus_nav_msgs::Twist2D>
{
protected:
  Twist2D normalized_goal_;
public:
  virtual ~SharedControlVelocityStrategy()
  {

  }

  virtual void init(const rtcus_dwa::DwaConfig& config, const Twist2D& goal)
  {
    VelocityStrategy<Twist2D, Twist2D>::init(config, goal);
    this->normalized_goal_ = *this->goal_;

    if (this->goal_->linear > this->config_->getKinodynamicConfig().linear_forward_speed_limit
        || this->goal_->linear < -this->config_->getKinodynamicConfig().linear_backwards_speed_limit
        || this->goal_->angular > this->config_->getKinodynamicConfig().angular_speed_limit
        || this->goal_->angular < -this->config_->getKinodynamicConfig().angular_speed_limit)
    {
      RTCUS_ASSERT_MSG(this->config_->getKinodynamicConfig().linear_backwards_speed_limit>=0,
                       "The backward speed should be defined positive and in m/s");
      dynamic_reconfigure::Config kinodesc_msg;
      this->config_->getKinodynamicConfig().__toMessage__(kinodesc_msg);
      ROS_WARN_STREAM(
          "The goal is being supplied outside from the kinematically feasible actions, trunking linear goal. GOAL: \n" << goal <<"\n Kinodynamic Description: \n" << kinodesc_msg);
      this->normalized_goal_.linear = max(
          min(this->goal_->linear, this->config_->getKinodynamicConfig().linear_forward_speed_limit),
          -this->config_->getKinodynamicConfig().linear_backwards_speed_limit);
      this->normalized_goal_.angular = max(
          min(this->goal_->angular, this->config_->getKinodynamicConfig().angular_speed_limit),
          -this->config_->getKinodynamicConfig().angular_speed_limit);
    }
  }

  virtual double computeCost(const CommandCost<rtcus_nav_msgs::Twist2D>& action)
  {
    RTCUS_ASSERT_MSG(action.getAction().linear >= 0, "This method is not prepared for backwards velocities");
    t_float velocity_error = fabs(
        (this->normalized_goal_.linear - action.getAction().linear)
            / this->config_->getKinodynamicConfig().linear_forward_speed_limit);

    RTCUS_ASSERT_MSG(
        velocity_error >= -0.0 && velocity_error <= 1.05,
        "action linear %lf, goal linear %lf -> error %lf", action.getAction().linear, this->normalized_goal_.linear, velocity_error);
    velocity_error = std::max(0.0, std::min(1.0, velocity_error));
    this->normalize(velocity_error);

    return velocity_error;
  }
};
}
}

#endif /* SHARED_CONTROL_VELOCITY_COST_STRATEGY_H_ */
