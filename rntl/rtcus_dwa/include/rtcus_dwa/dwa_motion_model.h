/*
 * dwa_motion_model.h
 *
 *  Created on: Jun 13, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef DWA_MOTION_MODEL_H_
#define DWA_MOTION_MODEL_H_

#include <rtcus_nav_msgs/DynamicState2D.h>
#include <rtcus_nav_msgs/Twist2D.h>
#include <rtcus_motion_models/motion_models/non_holonomic_2d.h>

namespace rtcus_dwa
{
using namespace rtcus_nav_msgs;
typedef rtcus_motion_models::DeterministicNonHolonomic2D<DynamicState2D, Twist2D> DWAMotionModel;
}

#endif
