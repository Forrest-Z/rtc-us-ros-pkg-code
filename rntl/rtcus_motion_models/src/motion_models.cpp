/*
 * motion_models.c
 *
 *  Created on: Mar 28, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 */

#include <mrpt_bridge/pose_conversions.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <rtcus_motion_models/motion_models.h>
#include <rtcus_motion_models/motion_models/motion_models_impl.h>
#include <rtcus_motion_models/motion_models/non_holonomic_2d.h>
#include <rtcus_motion_models/motion_models/stochastic_non_holonomic_motion_model.h>
#include <rtcus_nav_msgs/Twist2D.h>
#include <rtcus_nav_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <rtcus_nav_msgs/DynamicState2D.h>
#include <tf/tf.h>

namespace rtcus_motion_models
{

using namespace std;
using namespace mrpt::poses;
using namespace geometry_msgs;
using namespace rtcus_motion_models;
using namespace rtcus_nav_msgs;

void predictLocalStateTransition(double v, double omega, double dtt_secs, double &final_x, double& final_y,
                                 double& final_phi)
{
  if (fabs(omega) > 0.0)
  {
    double r = fabs(v / omega);
    //local frame icr
    // double icr_x = 0.0;

    if (v > 0)
      final_x = r * sin(fabs(omega) * dtt_secs);
    else
      final_x = -r * sin(fabs(omega) * dtt_secs);

    if (omega > 0)
    {
      double icr_y = r;
      final_y = icr_y - r * cos(omega * dtt_secs);
    }
    else
    {
      double icr_y = -r;
      final_y = icr_y + r * cos(omega * dtt_secs);
    }

    //this should be measured from -pi to pi
    final_phi = omega * dtt_secs;
    if (final_phi > M_PI)
      final_phi -= 2 * M_PI;
  }
  else
  {
    final_x = v * dtt_secs;
    final_y = 0.0;
    //this should be measured from -pi to pi
    final_phi = 0.0;
  }
}

template<>
  void DeterministicNonHolonomic2D<rtcus_nav_msgs::Pose2D, rtcus_nav_msgs::Twist2D>::predictLocalStateTransition(
      const rtcus_nav_msgs::Twist2D& action, const ros::Duration dt, rtcus_nav_msgs::Pose2D& final_state) const
  {
    rtcus_motion_models::predictLocalStateTransition((double)action.linear, (double)action.angular, dt.toSec(),
                                                     final_state.x, final_state.y, final_state.phi);
  }

template<>
  void DeterministicNonHolonomic2D<mrpt::poses::CPose2D, rtcus_nav_msgs::Twist2D>::predictLocalStateTransition(
      const rtcus_nav_msgs::Twist2D& action, const ros::Duration dt, mrpt::poses::CPose2D& final_state) const
  {
    rtcus_motion_models::predictLocalStateTransition(action.linear, action.angular, dt.toSec(), final_state[0],
                                                     final_state[1], final_state[2]);
  }

template<>
  void DeterministicNonHolonomic2D<Pose, Twist>::predictLocalStateTransition(const Twist& action,
                                                                             const ros::Duration dt,
                                                                             Pose& final_state) const
  {
    double phi;
    rtcus_motion_models::predictLocalStateTransition(action.linear.x, action.angular.z, dt.toSec(),
                                                     final_state.position.x, final_state.position.y, phi);
    final_state.position.z = 0;
    final_state.orientation = tf::createQuaternionMsgFromYaw(phi);
  }

template<>
  void DeterministicNonHolonomic2D<DynamicState2D, Twist2D>::predictLocalStateTransition(
      const Twist2D& action, const ros::Duration dt, DynamicState2D& final_state) const
  {
    double final_state_y, final_state_x, final_state_phi;
    rtcus_motion_models::predictLocalStateTransition(action.linear, action.angular, dt.toSec(), final_state_x,
                                                     final_state_y, final_state_phi);
    final_state.pose.x = final_state_x;
    final_state.pose.y = final_state_y;
    final_state.pose.phi = final_state_phi;
    final_state.twist.linear = action.linear;
    final_state.twist.angular = action.angular;
  }

template class MotionModel<Pose2D, Twist2D> ;
template class MotionModel<DynamicState2D, Twist2D> ;
template class MotionModel<Pose, Twist> ;
template class MotionModel<CPose2D, Twist2D> ;

template class DeterministicNonHolonomic2D<Pose2D, Twist2D> ;
template class DeterministicNonHolonomic2D<DynamicState2D, Twist2D> ;
template class DeterministicNonHolonomic2D<Pose, Twist> ;
template class DeterministicNonHolonomic2D<CPose2D, Twist2D> ;
}
