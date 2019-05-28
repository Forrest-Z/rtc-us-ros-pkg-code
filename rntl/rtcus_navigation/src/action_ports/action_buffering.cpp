/*
 * action_buffering.cpp
 *
 *  Created on: Apr 24, 2013
 *      Author: geus2
 */

#include <rtcus_navigation/action_ports/action_buffering.h>
#include <rtcus_nav_msgs/Twist2D.h>
#include <std_msgs/Float64.h>

namespace rtcus_navigation
{
namespace action_ports
{
using namespace rtcus_stamp;
template<typename ActionType>
  bool ActionBuffering<ActionType>::isEnabled() const
  {
    return this->config_.enabled;
  }

template<typename ActionType>
  ActionBuffering<ActionType>::ActionBuffering() :
      configure_server_(ros::NodeHandle("~/action_buffering"))
  {

    ros::NodeHandle nh;
    this->action_delay_pub_ = nh.advertise<std_msgs::Float64>("action_delays", 10);
    this->stamped_cmd_vel_topic_ = nh.subscribe<StampedAction>(
        "cmd_vel_stamped", 10, boost::bind(&ActionBuffering<ActionType>::stampedCmdvelReceived, this, _1));
    this->configure_server_.setCallback(boost::bind(&ActionBuffering<ActionType>::configuration_callback, this, _1));

  }
template<typename ActionType>
  ActionBufferingConfig ActionBuffering<ActionType>::configuration_callback(const ActionBufferingConfig& config)
  {
    boost::mutex::scoped_lock(msg_lock_);
    this->config_ = config;
    return this->config_;
  }

template<typename ActionType>
  inline bool less_than_key(const typename ActionBuffering<ActionType>::StampedAction& a,
                            const typename ActionBuffering<ActionType>::StampedAction& b)
  {
    return (a.header.stamp < b.header.stamp);
  }
template<typename ActionType>
  ros::Time ActionBuffering<ActionType>::getLastCmdTime() const
  {
    return this->t_u;
  }

template<typename ActionType>
  bool ActionBuffering<ActionType>::getCurrentAction(ActionType& action)
  {
    boost::mutex::scoped_lock(msg_lock_);
    //APPLY ACTION
    bool found = false;
    StampedAction ret;
    while (this->stamped_actions_buffer_.size() > 0)
    {
      const StampedAction& stored_action = this->stamped_actions_buffer_.front();
      if (stored_action.header.stamp.toSec() - this->config_.mechanical_delay < ros::Time::now().toSec())
      {
        this->t_u = ros::Time(ros::Time::now().toSec() + this->config_.mechanical_delay);
        //ROS_INFO("Appling old action stamped at %lf", stored_action.header.stamp.toSec());
        stamped_actions_buffer_.pop_front();
        ret = stored_action;
        found = true;
      }
      else
        break;
    }
    if (found)
    {
      action = rtcus_conversions::StampedConversion<ActionType>::remove_stamp(ret);
    }

    return found;
  }
// Message callback for a MsgBaseVel message, which set velocities.
template<typename ActionType>
  void ActionBuffering<ActionType>::stampedCmdvelReceived(const boost::shared_ptr<const StampedAction>& msg)
  {
    boost::mutex::scoped_lock(msg_lock_);
    if (this->config_.enabled)
    {
      std_msgs::Float64 delay;
      double now = ros::Time::now().toSec();
      double sec_diff = msg->header.stamp.toSec() - now;
      delay.data = sec_diff;
      this->action_delay_pub_.publish(delay);
      if (sec_diff > 0)
      {
        //ROS_WARN(
        //    "Action stored. Received at (%lf). wanted to be applied at (%lf)", ros::Time::now().toSec(), msg->header.stamp.toSec());
        this->stamped_actions_buffer_.push_back(*msg);
        this->stamped_actions_buffer_.sort(less_than_key<ActionType>);
      }
      else if (!this->config_.discard_old_commands || msg->header.stamp.toSec() == ros::Time::now().toSec())
      {

      }
      else
      {
        ROS_WARN(
            "Action Received at (%lf) discarded [discard_old_commands=true]. wanted to be applied at (%lf) [%lf secs late]", ros::Time::now().toSec(), msg->header.stamp.toSec(), ros::Time::now().toSec()- msg->header.stamp.toSec());
      }
    }
  }

template class ActionBuffering<rtcus_nav_msgs::Twist2D> ;
}

}

