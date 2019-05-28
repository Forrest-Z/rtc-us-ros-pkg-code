/*
 * deterministic_non_holonomic_motion_model.h
 *
 *  Created on: Jan 19, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *
 */

#ifndef DETERMINISTIC_TRAJECTORY_ROLLOUT_NON_HOLONOMIC_ICR_2D_H
#define DETERMINISTIC_TRAJECTORY_ROLLOUT_NON_HOLONOMIC_ICR_2D_H

#include <rtcus_motion_models/motion_models.h>
#include <rtcus_nav_msgs/DynamicState2D.h>
#include <rtcus_nav_msgs/Twist2D.h>
#include <rtcus_kinodynamic_description/NonHolonomicKinoDynamicsConfig.h>

namespace rtcus_motion_models
{

using namespace rtcus_nav_msgs;

class TrajectoryRolloutNonHolonomic2D : public MotionModel<DynamicState2D, Twist2D, ros::Duration>
{

public:

  typedef double t_float;
  /**
   * \brief The motion model can be built if the dynamic limits of the model are known.
   * */
  TrajectoryRolloutNonHolonomic2D(t_float time_integration_period,
                                  const rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig& kinoconfig);

  virtual ~TrajectoryRolloutNonHolonomic2D();
  void setKinodynamics(const rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig& kinoconfig);
  void setTimeIntegrationPeriod(t_float time_integration_period);
  inline t_float getTimeIntegrationPeriod() const
  {
    return this->time_integration_period_;
  }

  virtual void transfer_function(const DynamicState2D& initial_state, const Twist2D& action,
                                 const ros::Duration action_duration, DynamicState2D& final_state) const;

  void predictLocalStateTransition(t_float v, t_float omega, t_float u_v, t_float u_omega, t_float action_duration,
                                   t_float& final_v, t_float& final_omega, t_float &final_x, t_float& final_y,
                                   t_float& final_phi) const;

protected:
  rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig kinoconfig_;
  t_float time_integration_period_;
};

}

#endif /* DETERMINISTIC_NON_HOLONOMIC_ICR_2D_H */
