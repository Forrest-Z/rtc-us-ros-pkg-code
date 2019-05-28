/*
 * simulate_user_joystick_action_port.h
 *
 *  Created on: Feb 7, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef SIMULATE_USER_JOYSTICK_ACTION_PORT_H_
#define SIMULATE_USER_JOYSTICK_ACTION_PORT_H_

#include <rtcus_navigation/action_ports/ros_action_port_base.h>
#include <rtcus_nav_msgs/Twist2D.h>
#include <sensor_msgs/Joy.h>
#include <rtcus_navigation/kinodynamic_model.h>
#include <rtcus_kinodynamic_description/NonHolonomicKinoDynamicsConfig.h>

namespace rtcus_navigation
{
namespace action_ports
{
using namespace rtcus_nav_msgs;
using namespace sensor_msgs;
using namespace rtcus_navigation::action_ports;

class SimulateUserJoystickActionPort : public ROSActionPortBase<Twist2D, Joy, ROSTimeModel>
{
  USING_TIME_MODEL(ROSTimeModel);
protected:
  boost::shared_ptr<rtcus_navigation::KinoDynamicModel<rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig> > kino_model_;

  unsigned int linear_axis_;
  unsigned int angular_axis_;

  virtual void init();
  virtual void convertActionTypeToMsgType(const Twist2D& input, Joy& output, TTime expected_application_time);
  void onkinodynamicsChanged(const rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig&);
  rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig kinodesc_;
public:
  SimulateUserJoystickActionPort(
      boost::shared_ptr<
          rtcus_navigation::KinoDynamicModel<rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig> > kino_model);
  virtual ~SimulateUserJoystickActionPort();
};
}
}

#endif /* SIMULATE_USER_JOYSTICK_ACTION_PORT_H_ */
