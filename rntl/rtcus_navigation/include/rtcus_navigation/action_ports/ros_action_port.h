/*
 * default_ros_msg_action_port.h
 *
 *  Created on: Apr 17, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef DEFAULT_ROS_MSG_ACTION_PORT_H_
#define DEFAULT_ROS_MSG_ACTION_PORT_H_

#include <rtcus_navigation/action_ports/ros_action_port_base.h>
#include <rtcus_navigation/core.h>

namespace rtcus_navigation
{
namespace action_ports
{

using namespace rtcus_navigation::action_ports;

/**
 * \brief Use this action port, sending the action through a ROS Topic
 * \remarks Currently there are two version linked in this library, for geometry_msgs/Twist and for rtcus_nav_msgs/Twist2D
 */

template<typename ActionType, typename TimeModel = ROSTimeModel>
  class ROSActionPort : public ROSActionPortBase<ActionType, ActionType, TimeModel>
  {
    USING_TIME_MODEL(TimeModel);
  protected:
    virtual void convertActionTypeToMsgType(const ActionType& input, ActionType& output,
                                            TTime expected_application_time)
    {
      output = input;
    }
  public:
    virtual ~ROSActionPort()
    {
    }
  };
}
}
#endif /* DEFAULT_ROS_MSG_ACTION_PORT_H_ */
