/*
 * DWACommandCost.cpp
 *
 *  Created on: Dec 6, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_dwa/dwa_command_cost.h>
#include <iostream>

namespace rtcus_dwa
{

ostream& operator <<(ostream &out, rtcus_dwa::DwaCommandCost &command_cost)
{
  // Since operator<< is a friend of the Point class, we can access
  // Point's members directly.
  out << "Command [" << command_cost.getAction().linear << " , " << command_cost.getAction().angular << "] ["
      << command_cost.getTotalCost() << "] [ clearance: " << command_cost.getClearance() << " , heading: "
      << command_cost.getHeading() << ", velocity: " << command_cost.getVelocity() << "]";
  return out;
}

}

