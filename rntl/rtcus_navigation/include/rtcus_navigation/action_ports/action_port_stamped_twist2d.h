/*
 *
 *  Created on: Apr 17, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef ACTiON_PORT_STAMPED_TWIST2D
#define ACTiON_PORT_STAMPED_TWIST2D

#include <rtcus_navigation/action_ports/ros_action_port_base.h>
#include <rtcus_nav_msgs/Twist2D.h>
#include <rtcus_nav_msgs/StampedTwist2D.h>



namespace rtcus_navigation
{
namespace action_ports
{
using namespace rtcus_nav_msgs;
using namespace rtcus_navigation::action_ports;

class ActionPortStampedTwist2D : public ROSActionPortBase<Twist2D, StampedTwist2D, ROSTimeModel>
{
  USING_TIME_MODEL(ROSTimeModel);
protected:
  virtual void convertActionTypeToMsgType(const Twist2D& input, StampedTwist2D& output,
                                          TTime expected_application_time);
public:
  virtual ~ActionPortStampedTwist2D();
};

}
}
#endif /* DEFAULT_ROS_MSG_ACTION_PORT_H_ */
