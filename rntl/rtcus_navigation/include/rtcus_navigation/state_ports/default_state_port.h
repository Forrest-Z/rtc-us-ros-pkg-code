/*
 * default_localization_correction.h
 *
 *  Created on: Apr 10, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef DEFAULT_LOCALIZATION_CORRECTION_H_
#define DEFAULT_LOCALIZATION_CORRECTION_H_

#include <rtcus_navigation/state_port.h>
#include <rtcus_navigation/common.h>
#include <boost/thread.hpp>

#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

namespace rtcus_navigation
{
namespace state_ports
{
using namespace rtcus_stamp;

/**
 * \brief This is the default template type port. It can be templated to any kind of data structure stamped with header and timestamp
 * \remarks It always discard old, unordered messages (but not ordering them to avoid introducing any kind of delay).
 * */
template<typename StateType>
  class DefaultStatePort : public StatePort<StateType, ROSTimeModel>
  {
  protected:
    ros::Subscriber state_correction_sub_;
    std::string global_frame;
    StampedData<StateType> last_state;
    bool first_received;
    boost::mutex data_mutex_;

    void state_correctionCB(const boost::shared_ptr<StateType>& state_correction_msg)
    {
      std::string msg_frame;
      ros::Time state_time;

      StampedData<StateType> correction;

      if (!isHeadedRosMsg(state_correction_msg, state_time, msg_frame))
      {
        correction.copyFromData(*state_correction_msg);
        correction.setStamp(ros::Time::now());
        correction.setFrameId(global_frame.c_str());
      }
      else
      {
        if (ARE_DISTINCT_FRAMES(msg_frame, global_frame))
        {
          ROS_ERROR(
              "DefaultStatePort. State topic read which header frame [%s] does not match with frame [%s]", msg_frame.c_str(), global_frame.c_str());
          return;
        }
        correction.copyFromData(*state_correction_msg);
        correction.setStamp(state_time);
        correction.setFrameId(global_frame);
      }
      this->pushState(correction);
    }

  public:

    DefaultStatePort() :
        StatePort<StateType, ROSTimeModel>()
    {

    }
    virtual ~DefaultStatePort()
    {
    }

    virtual void init(std::string reference_frame)
    {
      reset();
      global_frame = reference_frame;
      last_state.setFrameId(global_frame);
    }

    virtual void reset()
    {
      last_state = StampedData<StateType>();
      first_received = false;
    }

    virtual void pushState(const StampedData<StateType>& correction)
    {

      if (!first_received)
      {
        ROS_INFO("State Port. First State Notification Received. State port is properly working.");
        first_received = true;
      }

      {
        boost::mutex::scoped_lock lock(data_mutex_);
        if (correction.getStamp() < last_state.getStamp())
        {
          ROS_ERROR(
              "DefauoltStatePort. the state correction is older than last state correction. State correction discarded.");
          return;
        }

        if (ARE_DISTINCT_FRAMES(correction.getFrameId(), global_frame))
        {
          ROS_ERROR(
              "DefauoltStatePort. State correction header frame [%s] does not match with frame [%s]", correction.getFrameId().c_str(), global_frame.c_str());
          return;
        }

        this->last_state.copyFrom(correction);
      }
      this->onStateReceived(*this, ros::Time::now());
    }

    virtual bool hasValidState() const
    {
      return first_received;
    }

    virtual void getLastState(rtcus_stamp::Stamped<StateType>& ret)
    {
      //allocates for  you if you are not passing allocated data. But be careful this could be
      //innefficent if the output parameter is a local scope variable.
      if (!ret.hasData())
        ret.setAllocatedData(boost::make_shared<StateType>());

      {
        boost::mutex::scoped_lock lock(data_mutex_);
        ret.copyFrom(last_state);
      }
    }

    virtual void copyLastState(rtcus_stamp::StampedData<StateType>& ret) const
    {
      //boost::mutex::scoped_lock lock(data_mutex_); <- is this ok? I can't because is a constant function member
      ret = last_state;
    }

  };

template class DefaultStatePort<nav_msgs::Odometry> ;
}
}
#endif /* DEFAULT_LOCALIZATION_CORRECTION_H_ */
