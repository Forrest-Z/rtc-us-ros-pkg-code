/*
 * dwa_motion_model.cpp
 *
 *  Created on: Jun 13, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */
#include <rtcus_dwa/dwa_motion_model.h>
#include <rtcus_compositions/impl/points.h>
#include <rtcus_navigation/state_ports/adaptable_state_correction_port.h>
#include <nav_msgs/Odometry.h>
#include <rtcus_motion_models/motion_models/motion_models_impl.h>

namespace rtcus_compositions
{
using namespace rtcus_dwa;
using namespace rtcus_nav_msgs;
using namespace rtcus_motion_models;

// FAKE DYNAMIC STATE COMPOSER SINCE IT WILL BE USED FOR THE KINEMATIC MOTION MODEL AND ONLY POSE VARIABLES
// ARE IMPORTANT
template<>
  void StateComposer::compose<DynamicState2D, DynamicState2D>(const DynamicState2D& state,
                                                              const DynamicState2D& transform, DynamicState2D& dst)
  {
    StateComposer::compose(state.pose, transform.pose, dst.pose);
    dst.twist = transform.twist;
  }

//FAKE DYNAMIC STATE COMPOSER
template<>
  void StateComposer::inverse_compose<DynamicState2D, DynamicState2D>(const DynamicState2D& src,
                                                                      const DynamicState2D& local_reference_frame,
                                                                      DynamicState2D& dst,
                                                                      const std::string& new_frame_name)
  {
    //HERE IT IS NEEDED A JACOBIAN
#pragma warning ("it is also needed to make the composition of the dynamic information. The dynamic information is not composed in this implementation")

    StateComposer::inverse_compose(src.pose, local_reference_frame.pose, dst.pose);
    //Supposing the local_reference_frame is static (not moving)
    dst.twist = src.twist;

  }
}
