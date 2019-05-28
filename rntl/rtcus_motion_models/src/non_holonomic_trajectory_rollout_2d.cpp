/*
 * non_holonomic_trajectory_rollout_2d.cpp
 *
 *  Created on: Jan 20, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 */

#include <rtcus_motion_models/motion_models/non_holonomic_trajectory_rollout_2d.h>
#include <rtcus_motion_models/motion_models/motion_models_impl.h>
#include <rtcus_assert/rtcus_assert.h>
#include <rtcus_compositions/state_composer.h>

namespace rtcus_motion_models
{

using namespace rtcus_nav_msgs;

template class MotionModel<DynamicState2D, Twist2D, ros::Duration> ;

void TrajectoryRolloutNonHolonomic2D::setKinodynamics(
    const rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig& kinoconfig)
{
  this->kinoconfig_ = kinoconfig;
}

void TrajectoryRolloutNonHolonomic2D::setTimeIntegrationPeriod(t_float time_integration_period)
{
  time_integration_period_ = time_integration_period;
}

TrajectoryRolloutNonHolonomic2D::TrajectoryRolloutNonHolonomic2D(
    t_float time_integration_period, const rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig& kinoconfig)
{
  time_integration_period_ = time_integration_period;
  this->kinoconfig_ = kinoconfig;
}

TrajectoryRolloutNonHolonomic2D::~TrajectoryRolloutNonHolonomic2D()
{
}

void TrajectoryRolloutNonHolonomic2D::transfer_function(const DynamicState2D& initial_state, const Twist2D& action,
                                                        const ros::Duration action_duration,
                                                        DynamicState2D& final_state) const
{
  DynamicState2D delta_state;

  this->predictLocalStateTransition(initial_state.twist.linear, initial_state.twist.angular, action.linear,
                                    action.angular, action_duration.toSec(), delta_state.twist.linear,
                                    delta_state.twist.angular, delta_state.pose.x, delta_state.pose.y,
                                    delta_state.pose.phi);
  final_state.twist = delta_state.twist;
  rtcus_compositions::StateComposer::compose(initial_state.pose, delta_state.pose, final_state.pose);
}

void TrajectoryRolloutNonHolonomic2D::predictLocalStateTransition(t_float v, t_float omega, t_float u_v,
                                                                  t_float u_omega, t_float action_duration,
                                                                  t_float& final_v, t_float& final_omega,
                                                                  t_float &final_x, t_float& final_y,
                                                                  t_float& final_phi) const
{
  /*RTCUS_ASSERT_MSG(this->time_integration_period_ <= action_duration,
   "The 'action application time' %lf should be greater than the 'time integration period' %lf",
   action_duration, time_integration_period_);*/

  t_float dt_ = std::min(this->time_integration_period_, action_duration);

  final_x = 0;
  final_y = 0;
  final_phi = 0;

  final_omega = omega;
  final_v = v;

  for (t_float t = 0; t < action_duration; t += dt_)
  {
    if (u_v >= final_v)
      final_v = std::min(std::min(final_v + this->kinoconfig_.linear_acceleration_limit * dt_, u_v),
                         this->kinoconfig_.linear_forward_speed_limit);
    else
      final_v = std::max(std::max(final_v - this->kinoconfig_.linear_brake_limit * dt_, u_v),
                         this->kinoconfig_.linear_backwards_speed_limit);

    if (u_omega >= omega)
      //final_omega = std::min(final_omega + this->kinoconfig_.angular_acceleration_limit * dt_, u_omega);
      final_omega = std::min(std::min(final_omega + this->kinoconfig_.angular_acceleration_limit * dt_, u_omega),
                             this->kinoconfig_.angular_speed_limit);
    else
      //final_omega = std::max(final_omega - this->kinoconfig_.angular_acceleration_limit * dt_, u_omega);
      final_omega = std::max(std::max(final_omega - this->kinoconfig_.angular_acceleration_limit * dt_, u_omega),
                             -this->kinoconfig_.angular_speed_limit);

    final_phi += final_omega * dt_;
    final_x += final_v * dt_ * cos(final_phi);
    final_y += final_v * dt_ * sin(final_phi);
  }
}

}
