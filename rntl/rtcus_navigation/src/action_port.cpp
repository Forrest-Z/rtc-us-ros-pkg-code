/*
 * action_port.cpp
 *
 *  Created on: Apr 15, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 */

#include <rtcus_navigation/action_port.h>
#include <rtcus_navigation/action_ports/ros_action_port.h>
#include <rtcus_navigation/action_ports/adaptable_action_port.h>

namespace rtcus_navigation
{
namespace action_ports
{
template class ROSActionPort<rtcus_nav_msgs::Twist2D> ;
template class ROSActionPort<geometry_msgs::Twist> ;

}
}
;
