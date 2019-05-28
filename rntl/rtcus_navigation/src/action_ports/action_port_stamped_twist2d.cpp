/*
 * action_port_stamped_twist2d.cpp
 *
 *  Created on: Dec 15, 2012
 *      Author: root
 */

#include <rtcus_navigation/action_ports/action_port_stamped_twist2d.h>

using namespace rtcus_nav_msgs;

namespace rtcus_navigation
{
namespace action_ports
{

using namespace rtcus_navigation::action_ports;

void ActionPortStampedTwist2D::convertActionTypeToMsgType(const Twist2D& input, StampedTwist2D& output,
                                                          TTime expected_application_time)
{
  output.twist = input;
  output.header.stamp = expected_application_time;

}

ActionPortStampedTwist2D::~ActionPortStampedTwist2D()
{
}

}
}
