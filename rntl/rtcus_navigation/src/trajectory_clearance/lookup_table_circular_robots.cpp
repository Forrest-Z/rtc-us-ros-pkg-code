/*
 * lookup_table_circular_robots.cpp
 *
 *  Created on: Dec 12, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_navigation/trajectory_clearance/circular_trajectory_clearance_circular_robot.h>
#include <rtcus_navigation/trajectory_clearance/trajectory_clearance_lookup_table_decorator.h>
#include <rtcus_robot_shapes/circular_robot.h>
#include <rtcus_nav_msgs/Twist2D.h>

using namespace rtcus_navigation::trajectory_clearance;
using namespace rtcus_robot_shapes::interfaces;

namespace rtcus_navigation
{
namespace trajectory_clearance
{
  template class TrajectoryClearanceLookupTable2D<Twist2D, ICircularRobotShape> ;
}
}
