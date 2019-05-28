/*
 * deterministic_non_holonomic_motion_model.h
 *
 *  Created on: Apr 10, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *
 */

#ifndef DETERMINISTIC_NON_HOLONOMIC_ICR_2D_H
#define DETERMINISTIC_NON_HOLONOMIC_ICR_2D_H

#include <rtcus_nav_msgs/Pose2DWithCovariance.h>
#include <rtcus_nav_msgs/Twist2D.h>
#include <rtcus_motion_models/motion_models.h>
#include <rtcus_motion_models/motion_models/motion_models_impl.h>

namespace rtcus_motion_models
{

using namespace rtcus_nav_msgs;
class ProbabilisticNonholonomicVelocityMotionModel : public MotionModel<Pose2DWithCovariance, Twist2D>
{
private:
  typedef double t_float;

protected:
  virtual void transfer_function(const Pose2DWithCovariance& initial_state, const Twist2D& action, const ros::Duration dt,
                                 Pose2DWithCovariance& final_state) const
  {
    // p(v_|v,omega)= N (v, Sigma(a1*v + a2*omega))
    // p(omega_|v,omega)= N(omega, Sigma(a3*v + a4*omega))
    // B (v,omega,t)-> [x,y,phi]


    // B must include fi= omega t + phinoise
    // phinoise = p(fi_ | fi)= N (fi, Sigma(a5*v + a6*omega))
    // -> delta_state
    // Jacob B in u=action t= dt




    //// HARDCODE MATRIX JACOBIAN  f_composition(q_a, q_b ) in q_a=initial_state , q_b=delta_state
    //lets J
    //final_state.covariance = J* aggregate(initial_state.cov, action.cov, dtdev)* J';


  }

public:
  ProbabilisticNonholonomicVelocityMotionModel()
  {

  }
  virtual ~ProbabilisticNonholonomicVelocityMotionModel()
  {
  }

};

}

#endif /* DETERMINISTIC_NON_HOLONOMIC_ICR_2D_H */
