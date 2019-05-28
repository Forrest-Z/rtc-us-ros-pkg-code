/*
 * simulate_user_joystick_action_port.cpp
 *
 *  Created on: Feb 7, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_navigation/action_ports/simulate_user_joystick_action_port.h>
//TODO: Change this include file name which is not appropriate. Remove the word:"static"
#include <rtcus_navigation/kinodynamic_models/configurable_static_robot_kinodynamics.h>
#include <rtcus_navigation/kinodynamic_models/default_static_robot_kinodynamics.h>
#include <math.h>
#include <rtcus_nav_msgs/Twist2D.h>

using namespace sensor_msgs;
using namespace rtcus_navigation::action_ports;
using namespace rtcus_navigation::kinodynamic_models;
using namespace boost;
using namespace std;
using namespace rtcus_nav_msgs;

namespace rtcus_navigation
{
namespace action_ports
{

SimulateUserJoystickActionPort::SimulateUserJoystickActionPort(
    boost::shared_ptr<rtcus_navigation::KinoDynamicModel<rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig> > kino_model) :
    ROSActionPortBase<Twist2D, Joy, ROSTimeModel>::ROSActionPortBase("joy"), linear_axis_(1), angular_axis_(0)
{
  this->kino_model_ = kino_model;
}

void SimulateUserJoystickActionPort::init()
{
  if (this->kino_model_ == NULL)
  {
    std::string dynamic_reconfigure_uri = this->getComponentNode().getNamespace() + "/kinodynamic_model";
    ROS_INFO(
        "%s.  A new kinodynamic specification at %s", getClassName(*this).c_str(), dynamic_reconfigure_uri.c_str());

    this->kino_model_ = shared_ptr<
        ConfigurableKinodynamicModel<rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig> >(
        new ConfigurableKinodynamicModel<rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig>(
            make_shared<DefaultKinodynamicModel<rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig> >(),
            ros::NodeHandle(dynamic_reconfigure_uri)));

    this->kino_model_->init();
  }
  RTCUS_ASSERT(this->kino_model_!=NULL);
  this->kino_model_->onKinodynamicsChanged.connect(
      boost::bind(&SimulateUserJoystickActionPort::onkinodynamicsChanged, this, _1));
}

void SimulateUserJoystickActionPort::onkinodynamicsChanged(
    const rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig& kinodesc)
{

  RTCUS_ASSERT_MSG(kinodesc.linear_backwards_speed_limit>=0,
                   "backward speed should be defined positively in meters per seconds");
  dynamic_reconfigure::Config message;
  kinodesc.__toMessage__(message);
  ROS_INFO_STREAM(" * " <<getClassName(*this) << " - Kinodynamic description has been updated: \n "<<message);
  this->kinodesc_ = kinodesc;
}

void SimulateUserJoystickActionPort::convertActionTypeToMsgType(const Twist2D& input, Joy& output,
                                                                TTime expected_application_time)
{
  output.header.stamp = ros::Time::now();
  RTCUS_ASSERT_MSG(
      input.linear >= - this->kinodesc_.linear_backwards_speed_limit && input.linear <= this->kinodesc_.linear_forward_speed_limit,
      "linear command %lf, linear speed limit: %lf, back linear speed limit %lf", input.linear, this->kinodesc_.linear_forward_speed_limit, - this->kinodesc_.linear_backwards_speed_limit);

  double xj = 0, yj = 0;
  //polar action space coordinates
  float alpha = atan2(input.linear, input.angular);
  Twist2D normalized_goal;
  normalized_goal.angular = input.angular / this->kinodesc_.angular_speed_limit;
  if (input.linear >= 0)
    normalized_goal.linear = input.linear / this->kinodesc_.linear_forward_speed_limit;
  else
    normalized_goal.linear = input.linear / this->kinodesc_.linear_backwards_speed_limit;

  float mag = sqrt(normalized_goal.linear * normalized_goal.linear + normalized_goal.angular * normalized_goal.angular);

  float den = fabs(mag * cos(alpha));
  if (den != 0)
    xj = input.angular / den;
  else
    xj = 0.0;

  den = fabs(mag * sin(alpha));
  if (den != 0)
    yj = input.linear / den;
  else
    yj = 0.0;

  output.axes = std::vector<float>(2);
  output.axes[this->linear_axis_] = min(1.0, max(-1.0, yj));
  output.axes[this->angular_axis_] = min(1.0, max(-1.0, xj));

  /*
   float v_axis;
   float h_axis;
   if (fabs(input.linear) > 0.0)
   {

   float alpha_k = atan2(input.linear, input.angular);

   if (input.linear >= 0)
   {
   float mag = sqrt(input.linear * input.linear + input.angular * input.angular)
   / sqrt(
   kinodesc.linear_forward_speed_limit * kinodesc.linear_forward_speed_limit
   + kinodesc.angular_speed_limit * kinodesc.angular_speed_limit);
   v_axis = input.linear / kinodesc.linear_forward_speed_limit;
   }
   else
   {
   float mag = sqrt(input.linear * input.linear + input.angular * input.angular)
   / sqrt(
   kinodesc.linear_backwards_speed_limit * kinodesc.linear_backwards_speed_limit
   + kinodesc.angular_speed_limit * kinodesc.angular_speed_limit);
   v_axis = input.linear / kinodesc.linear_backwards_speed_limit;
   }

   h_axis = cos(alpha_k) * kinodesc.angular_speed_limit;
   }
   else
   {

   }
   */

}
SimulateUserJoystickActionPort::~SimulateUserJoystickActionPort()
{
}
}
;
}

