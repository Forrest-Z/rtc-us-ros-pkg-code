/*
 * stochastic_non_holonomic_motion_model.h
 *
 *  Created on: Apr 10, 2012
 *      Author: geus
 */

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovariance.h>

namespace rtcus_motion_models
{

// IMPLEMENT FROM THIS : http://www.mrpt.org/Probabilistic_Motion_Models
class StocasticNonHolonomicStateEstimator2D
/*: public DeterministicNonHolonomicStateEstimator2D, public MotionModel<
 mrpt::poses::CPose3DPDFGaussian, geometry_msgs::Twist>
 , public MotionModel<geometry_msgs::PoseWithCovariance,
 geometry_msgs::Twist>*/
{
  virtual ~StocasticNonHolonomicStateEstimator2D()
  {

  }
  virtual void step_prediction(const geometry_msgs::Twist& last_command, ros::Duration dt,
                               geometry_msgs::PoseWithCovariance& predicted_state) const
  {

  }
};

}
