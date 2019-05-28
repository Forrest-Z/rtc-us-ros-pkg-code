/*
 * state_estimation.cpp
 *
 *  Created on: Oct 2, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_navigation/state_estimation/default_state_estimation.h>
#include <rtcus_navigation/state_estimation/default_state_estimation_impl.h>

#include <rtcus_navigation/state_estimation/non_time_correction_state_estimation.h>
#include <rtcus_navigation/state_estimation.h>
#include <rtcus_motion_models/motion_models/non_holonomic_2d.h>
#include <rtcus_nav_msgs/DynamicState2D.h>
#include <rtcus_nav_msgs/Twist2D.h>
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>

namespace rtcus_navigation
{

namespace state_estimation
{
using namespace rtcus_nav_msgs;

/*Look for a more proper name without templates*/
class non_time_DynamicState2D_Twist2D : public rtcus_navigation::state_estimation::NonTimeCorrectionStateEstimation<
    DynamicState2D, Twist2D>
{
};


class default_nonholonomic_DynamicState2D_Twist2D : public DefaultStateEstimation<DynamicState2D, Twist2D, ROSTimeModel>
{
public:
  default_nonholonomic_DynamicState2D_Twist2D()
  {
    this->setMotionModel(make_shared<rtcus_motion_models::DeterministicNonHolonomic2D<DynamicState2D, Twist2D> >());
  }
};

/*register them as plugins*/
PLUGINLIB_DECLARE_CLASS (rtcus_navigation, default_nonholonomic_DynamicState2D_Twist2D,
    rtcus_navigation::state_estimation::default_nonholonomic_DynamicState2D_Twist2D,
    rtcus_navigation::NavigationNodeComponent)

PLUGINLIB_DECLARE_CLASS(rtcus_navigation, non_time_DynamicState2D_Twist2D,
    rtcus_navigation::state_estimation::non_time_DynamicState2D_Twist2D,
    rtcus_navigation::NavigationNodeComponent)

}}
