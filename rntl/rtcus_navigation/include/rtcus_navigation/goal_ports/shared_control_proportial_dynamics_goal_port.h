/*
 * shared_control_proportial_dynamics_goal_port.h
 *
 *  Created on: Jan 12, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef SHARED_CONTROL_PROPORTIAL_DYNAMICS_GOAL_PORT_H_
#define SHARED_CONTROL_PROPORTIAL_DYNAMICS_GOAL_PORT_H_

#include <rtcus_navigation/goal_ports/adaptable_goal_port.h>
#include <sensor_msgs/Joy.h>
#include <rtcus_conversions/conversions.h>
#include <boost/signal.hpp>
#include <rtcus_kinodynamic_description/NonHolonomicKinoDynamicsConfig.h>
#include <rtcus_navigation/JoyGoalPortAngularMappingConfig.h>
#include <dynamic_reconfigure/server.h>

namespace rtcus_navigation
{
namespace goal_ports
{

/**
 * \brief this component takes a joystick command as the navigation system goal. This is the default port for semi-operated obstacle
 * avoidance systems.
 * */
//TODO: move the code to a linkable unit cpp
class SharedControlProportionalDynamicsGoalPort : public AdaptableGoalPort<rtcus_nav_msgs::Twist2D, sensor_msgs::Joy>
{
protected:
  int linear_axis_;
  int angular_axis_;
  std::string mapping_rule_;
  rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig kinodynamic_config_;

  boost::shared_ptr<dynamic_reconfigure::Server<JoyGoalPortAngularMappingConfig> > lci_config_server_;
  JoyGoalPortAngularMappingConfig lci_mapping_config_;

  JoyGoalPortAngularMappingConfig config_callback(const JoyGoalPortAngularMappingConfig& config, int level);

public:
  virtual ~SharedControlProportionalDynamicsGoalPort();
  //TODO: remove this template constructor and use oninit of the goal port with AbstractNavigationNode
  template<typename TNavigationNode>
    SharedControlProportionalDynamicsGoalPort(TNavigationNode& nn) :
        linear_axis_(1), angular_axis_(0), mapping_rule_("angular")
    {
      this->reset();
      nn.getKinodynamicModel()->onKinodynamicsChanged.connect(
          boost::bind(&SharedControlProportionalDynamicsGoalPort::updateLimits, this, _1));

    }

  virtual void reset();
  void updateLimits(const rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig& config);
  virtual void process_message(StampedData<rtcus_nav_msgs::Twist2D>& action_target_goal,
                               const boost::shared_ptr<const sensor_msgs::Joy>& joy_msg);
  virtual void convert_msg(const sensor_msgs::Joy& joy_msg, rtcus_nav_msgs::Twist2D& action_target_goal);
};
}
}

#endif /* SHARED_CONTROL_PROPORTIAL_DYNAMICS_GOAL_PORT_H_ */
