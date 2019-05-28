/*
 * shared_control_proportional_dynamics.cpp
 *
 *  Created on: Jan 21, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_navigation/goal_ports/shared_control_proportial_dynamics_goal_port.h>
namespace rtcus_navigation
{
namespace goal_ports
{

SharedControlProportionalDynamicsGoalPort::~SharedControlProportionalDynamicsGoalPort()
{

}
void SharedControlProportionalDynamicsGoalPort::reset()
{
  AdaptableGoalPort<rtcus_nav_msgs::Twist2D, sensor_msgs::Joy>::reset();
  ROS_INFO(" * Initializating shared control angular mapping goal port.");
  if (!this->component_node_.getParam("axis_linear", linear_axis_))
    this->component_node_.setParam("axis_linear", linear_axis_);

  if (!this->component_node_.getParam("axis_angular", angular_axis_))
    this->component_node_.setParam("axis_angular", angular_axis_);

  if (!this->component_node_.getParam("mapping_rule", mapping_rule_))
    this->component_node_.setParam("mapping_rule", mapping_rule_);

  if (!this->component_node_.getParam("angular_mapping/forward_bias", this->lci_mapping_config_.forward_bias))
    this->component_node_.setParam("angular_mapping/forward_bias", this->lci_mapping_config_.forward_bias);

  if (this->lci_config_server_)
    this->lci_config_server_->clearCallback();

  ROS_INFO(" * Creating the dynamic_reconfigure server");
  this->lci_config_server_ = boost::shared_ptr<dynamic_reconfigure::Server<JoyGoalPortAngularMappingConfig> >(
      new dynamic_reconfigure::Server<JoyGoalPortAngularMappingConfig>(
          this->getComponentNode().getNamespace() + "/angular_mapping"));

  this->lci_config_server_->setCallback(
      boost::bind(&SharedControlProportionalDynamicsGoalPort::config_callback, this, _1, _2));

}

void SharedControlProportionalDynamicsGoalPort::updateLimits(
    const rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig& config)
{
  this->kinodynamic_config_ = config;
}

void SharedControlProportionalDynamicsGoalPort::process_message(
    StampedData<rtcus_nav_msgs::Twist2D>& action_target_goal, const boost::shared_ptr<const sensor_msgs::Joy>& joy_msg)
{
  action_target_goal.setFrameId(robot_base_local_frame_);
  action_target_goal.setStamp(joy_msg->header.stamp);
  this->convert_msg(*joy_msg, action_target_goal.getData());
}

JoyGoalPortAngularMappingConfig SharedControlProportionalDynamicsGoalPort::config_callback(
    const JoyGoalPortAngularMappingConfig& config, int level)
{
  this->lci_mapping_config_ = config;
  return lci_mapping_config_;
}

void SharedControlProportionalDynamicsGoalPort::convert_msg(const sensor_msgs::Joy& joy_msg,
                                                            rtcus_nav_msgs::Twist2D& action_target_goal)
{
  float yj = joy_msg.axes[this->linear_axis_];
  float xj = joy_msg.axes[this->angular_axis_];

  double alpha;
  if (yj == xj && xj == 0)
    alpha = M_PI_2;
  else
    alpha = atan2(yj * lci_mapping_config_.forward_bias, xj);

  double mag = sqrt(yj * yj + xj * xj);
  if (joy_msg.buttons[0] == 1)
    mag = yj = 0.0;
  else if (joy_msg.buttons[1] == 1)
    mag = yj = 1.0;

  double v_prop;
  if (yj >= 0.0)
    v_prop = yj * this->kinodynamic_config_.linear_forward_speed_limit;
  else
    v_prop = yj * this->kinodynamic_config_.linear_backwards_speed_limit;

  if (mapping_rule_ == "angular")
  {
    action_target_goal.linear = std::min(
        std::max(mag * sin(alpha) * fabs(v_prop), -this->kinodynamic_config_.linear_backwards_speed_limit),
        this->kinodynamic_config_.linear_forward_speed_limit);

    action_target_goal.angular = std::min(
        std::max(mag * cos(alpha) * fabs(xj * this->kinodynamic_config_.angular_speed_limit),
                 -this->kinodynamic_config_.angular_speed_limit),
        this->kinodynamic_config_.angular_speed_limit);
    action_target_goal.lateral = 0.0;
  }
  else if (mapping_rule_ == "proportional")
  {
    action_target_goal.linear = v_prop;
    action_target_goal.angular = xj * this->kinodynamic_config_.angular_speed_limit;
  }

  /*//proportional rule for the linear velocity
   if (linear_joy_cmd >= 0) //forward
   action_target_goal.linear = linear_joy_cmd * this->kinodynamic_config_.linear_forward_speed_limit;
   else //backward
   {
   RTCUS_ASSERT_MSG(this->kinodynamic_config_.linear_backwards_speed_limit >= 0,
   "The backward speed should be specified unsigned");
   //backwards
   action_target_goal.linear = linear_joy_cmd * this->kinodynamic_config_.linear_backwards_speed_limit;
   }

   if (linear_joy_cmd == 0 && angular_joy_cmd == 0)
   {
   action_target_goal.linear = 0;
   action_target_goal.lateral = 0;
   action_target_goal.angular = 0;
   }
   else if (linear_joy_cmd == 0 && angular_joy_cmd != 0)
   {
   action_target_goal.linear = 0;
   action_target_goal.lateral = 0;
   action_target_goal.angular = angular_joy_cmd * this->kinodynamic_config_.angular_speed_limit;
   }
   else
   {
   float joy_kurvature_alpha = atan2(linear_joy_cmd, angular_joy_cmd);
   action_target_goal.angular = action_target_goal.linear * cos(joy_kurvature_alpha) / sin(linear_joy_cmd);
   }*/
}
}
;
}

