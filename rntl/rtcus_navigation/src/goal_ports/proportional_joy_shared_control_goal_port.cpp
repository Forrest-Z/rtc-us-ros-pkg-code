/*
 * shared_control_goal_port.cpp
 *
 *  Created on: Jan 21, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_navigation/goal_ports/proportional_joy_shared_control_goal_port.h>

namespace rtcus_navigation
{
namespace goal_ports
{
ProportionalJoySharedControlGoalPort::~ProportionalJoySharedControlGoalPort()
{

}
ProportionalJoySharedControlGoalPort::ProportionalJoySharedControlGoalPort() :
    omega_limit_(1.0), v_forwards_limit_(1.0), linear_axis_(1), angular_axis_(0)
{
  if (!this->component_node_.getParam("axis_linear", linear_axis_))
    this->component_node_.setParam("axis_linear", linear_axis_);

  if (!this->component_node_.getParam("axis_angular", angular_axis_))
    this->component_node_.setParam("axis_angular", angular_axis_);
}
void ProportionalJoySharedControlGoalPort::updateLimits(double v_forwards_limit, double omega_limit)
{
  this->omega_limit_ = omega_limit;
  this->v_forwards_limit_ = v_forwards_limit;
}

void ProportionalJoySharedControlGoalPort::process_message(StampedData<rtcus_nav_msgs::Twist2D>& action_target_goal,
                                            const boost::shared_ptr<const sensor_msgs::Joy>& joy_msg)
{
  action_target_goal.setFrameId(robot_base_local_frame_);
  action_target_goal.setStamp(joy_msg->header.stamp);
  this->convert_msg(*joy_msg, action_target_goal.getData());
}
void ProportionalJoySharedControlGoalPort::convert_msg(const sensor_msgs::Joy& joy_msg, rtcus_nav_msgs::Twist2D& action_target_goal)
{
  action_target_goal.linear = joy_msg.axes[this->linear_axis_] * v_forwards_limit_;
  action_target_goal.angular = joy_msg.axes[this->angular_axis_] * omega_limit_;

}

}
}
