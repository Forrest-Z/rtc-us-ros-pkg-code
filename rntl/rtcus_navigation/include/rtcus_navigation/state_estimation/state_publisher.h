/*
 * state_publisher.h
 *
 *  Created on: Dec 26, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef STATE_PUBLISHER_H_
#define STATE_PUBLISHER_H_

#include <ros/ros.h>
#include <rtcus_conversions/conversions_stamped.h>
#include <rtcus_stamp/stamped.h>

namespace rtcus_navigation
{
template<typename StateType>
  class StatePublisher
  {
    ros::Publisher state_pub_;
    typedef typename rtcus_conversions::StampedConversion<StateType>::StampedType StampedType;
  public:
    StatePublisher(ros::NodeHandle& nh, std::string topic_name)
    {
      state_pub_ = nh.advertise<StampedType>(topic_name, 10);
    }
    void publish(const Stamped<StateType>& m)
    {
      StampedType m2;
      rtcus_conversions::StampedConversion<StateType>::convert(m, m2);
      state_pub_.publish(m2);
    }
  };

}

#endif /* STATE_PUBLISHER_H_ */
