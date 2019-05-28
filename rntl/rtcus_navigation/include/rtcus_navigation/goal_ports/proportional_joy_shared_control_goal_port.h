/*
 * shared_control_goal_port.h
 *
 *  Created on: Oct 25, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef PROPORTIONAL_JOY_SHARED_CONTROL_GOAL_PORT_H_
#define PROPORTIONAL_JOY_SHARED_CONTROL_GOAL_PORT_H_

#include <rtcus_navigation/goal_ports/adaptable_goal_port.h>
#include <sensor_msgs/Joy.h>
#include <rtcus_conversions/conversions.h>
#include <boost/signal.hpp>

namespace rtcus_navigation
{
namespace goal_ports
{
class ProportionalJoySharedControlGoalPort : public AdaptableGoalPort<rtcus_nav_msgs::Twist2D, sensor_msgs::Joy>
{
protected:
  /* \brief Proportionality constant for angular speed*/
  double omega_limit_;
  /* \brief Proportionality constant for linear speed*/
  double v_forwards_limit_;

  int linear_axis_;
  int angular_axis_;

public:
  virtual ~ProportionalJoySharedControlGoalPort();
  ProportionalJoySharedControlGoalPort();
  void updateLimits(double v_forwards_limit, double omega_limit);
  virtual void process_message(StampedData<rtcus_nav_msgs::Twist2D>& action_target_goal,
                               const boost::shared_ptr<const sensor_msgs::Joy>& joy_msg);
  virtual void convert_msg(const sensor_msgs::Joy& joy_msg, rtcus_nav_msgs::Twist2D& action_target_goal);
};
}
}
#endif /* SHARED_CONTROL_GOAL_PORT_H_ */
