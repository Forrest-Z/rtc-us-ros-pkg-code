/*
 *
 *  Created on: Apr 17, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef ADAPTABLE_ACTION_PORT_H_
#define ADAPTABLE_ACTION_PORT_H_

#include <rtcus_navigation/action_port.h>
#include <rtcus_navigation/action_ports/ros_action_port_base.h>
#include <rtcus_conversions/conversions.h>

namespace rtcus_navigation
{
namespace action_ports
{
using namespace rtcus_navigation::action_ports;

template<typename ActionType, typename RosMsgType, typename TimeModel = ROSTimeModel>
  class ROSAdaptableActionPort : public ROSActionPortBase<ActionType, RosMsgType, TimeModel>
  {
    USING_TIME_MODEL(TimeModel);

  protected:
    virtual void convertActionTypeToMsgType(const ActionType& input, RosMsgType& output,
                                            TTime expected_application_time)
    {
      rtcus_conversions::Conversions::convert(input, output);
      if (ros::message_traits::HasHeader<RosMsgType>::value)
      {

        std_msgs::Header* h = ros::message_traits::Header<RosMsgType>::pointer(output);
        h->stamp = expected_application_time;
      }
    }
  public:
    virtual ~ROSAdaptableActionPort()
    {
    }

  };

}
}

#endif /* DEFAULT_ROS_MSG_ACTION_PORT_H_ */
