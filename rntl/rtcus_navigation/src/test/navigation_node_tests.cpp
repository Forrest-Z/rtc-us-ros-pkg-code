/*
 * world_perception_tests.cpp
 *
 *  Created on: Apr 10, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <gtest/gtest.h>
#include <rtcus_navigation/navigation_node.h>
#include <rtcus_nav_msgs/DynamicState2D.h>
#include <rtcus_compositions/state_composer.h>

using namespace rtcus_nav_msgs;
using namespace rtcus_navigation;

namespace rtcus_compositions
{
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


TEST(NavigationNodeTest, BasicInitialization)
{

}

TEST(NavigationNodeTest, BasicInitialization2)
{

}
