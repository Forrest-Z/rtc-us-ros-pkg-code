/*
 * high_noise_odometry_state_port.cpp
 *
 *  Created on: Jan 29, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_navigation/state_ports/high_noise_odometry_state_port.h>

namespace rtcus_navigation
{
namespace state_ports
{

const HighNoiseOdometryStatePortConfig& HighNoiseOdometryStatePort::configure_server_callback(
    HighNoiseOdometryStatePortConfig &config)
{
  boost::mutex::scoped_lock lock(m_mutex);
  ROS_DEBUG("%s. Configure server Callback.", getClassName(*this).c_str());
  this->config_ = config;
  return this->config_;
}

void HighNoiseOdometryStatePort::fromInputRosMsgToInternalState(const nav_msgs::Odometry& msg,
                                                                DynamicState2D& output_state, ros::Time& output_stamp,
                                                                std::string& frame_id)
{
  boost::mutex::scoped_lock lock(m_mutex);
  ROS_DEBUG("%s. convert from input message.", getClassName(*this).c_str());
  StampedData<DynamicState2D> current_reading;
  rtcus_conversions::Conversions::convert(msg, *current_reading);
  current_reading.setStamp(msg.header.stamp);
  current_reading.setFrameId(msg.header.frame_id);

  StampedData<DynamicState2D> out;
  compute_mean_and_pop(current_reading, out);
  output_state = *out;
  output_stamp = out.getStamp();
  frame_id = out.getFrameId();
}

inline float angle_dist(float phi1, float phi2)
{
  static const float twopi = 2 * M_PI;

  if (phi1 < 0)
    phi1 += twopi;
  else if (phi1 > twopi)
    phi1 -= twopi;

  if (phi2 < 0)
    phi2 += twopi;
  else if (phi2 > twopi)
    phi2 -= twopi;

  float dist = phi1 - phi2;
  float absdist = fabs(dist);
  if (absdist > M_PI)
    absdist = twopi - absdist;

  if (dist > 0)
    return absdist;
  else
    return -absdist;

}

void HighNoiseOdometryStatePort::compute_mean_and_pop(const StampedData<DynamicState2D>& newreading,
                                                      StampedData<DynamicState2D>& out)
{
  ROS_DEBUG("* %s. new reading at %lf", getClassName(*this).c_str(), newreading.getStamp().toSec());
  ros::Time now = ros::Time::now();
  ros::Duration interval = ros::Duration(this->config_.time_buffer);
  ROS_DEBUG("* %s. cleaning buffer earlier than %lf", getClassName(*this).c_str(), (now -interval).toSec());
  while (this->state_info_buffer_.size() > 0 && now - this->state_info_buffer_.front().getStamp() > interval)
  {
    ROS_DEBUG(
        "* %s. Removing reading at %lf", getClassName(*this).c_str(), this->state_info_buffer_.front().getStamp().toSec());
    this->state_info_buffer_.pop_front();
  }

  //APPEND NEW READING TO THE BUFFER
  if (now - newreading.getStamp() < interval // verifies the interval
  && (this->state_info_buffer_.size() == 0 //buffer is empty or
  || (this->state_info_buffer_.back().getStamp() < newreading.getStamp()))) //data is ordered
    this->state_info_buffer_.push_back(newreading);

  if (this->state_info_buffer_.size() == 0)
  {
    ROS_DEBUG("* %s. Buffer empty, usipng last reading", getClassName(*this).c_str());
    out = newreading;
    //if is not too much old append reading
    return;
  }
  else //------------------- COMPUTING MEAN------------------------------------
  {
    ROS_DEBUG(
        "* %s. computing mean for (%ld) state readings between (%lf) and (%lf)", getClassName(*this).c_str(), this->state_info_buffer_.size(), this->state_info_buffer_.front().getStamp().toSec(), this->state_info_buffer_.back().getStamp().toSec());
//    float time_mean = 0;
    StampedData<DynamicState2D> state_mean;
    state_mean->pose.x = state_mean->pose.y = state_mean->pose.phi = state_mean->twist.linear =
        state_mean->twist.angular = state_mean->twist.lateral = 0;

    //  float phi_ref = this->state_info_buffer_.front()->pose.phi;

    BOOST_FOREACH(const StampedData<DynamicState2D>& current, this->state_info_buffer_)
    {
      //time_mean+=current.getStamp().toSec();
      //state_mean->pose.x+=current->pose.x;
      //state_mean->pose.y+=current->pose.y;
      //state_mean->pose.phi+= phi_ref + angle_dist(phi_ref,current->pose.phi);

      state_mean->twist.linear+=current->twist.linear;
      state_mean->twist.lateral+=current->twist.lateral;
      state_mean->twist.angular+=current->twist.angular;
    }

    //state_mean->pose.x /= ((float)this->state_info_buffer_.size());
    //state_mean->pose.y /= ((float)this->state_info_buffer_.size());
    //state_mean->pose.phi /= ((float)this->state_info_buffer_.size());
    //state_mean->pose.phi = state_info_buffer_.back()->pose.phi;
    //state_mean.setStamp(ros::Time(time_mean / (float)this->state_info_buffer_.size()));

    state_mean.setStamp(this->state_info_buffer_.back().getStamp());
    state_mean.setFrameId(this->state_info_buffer_.back().getFrameId());
    state_mean->pose = this->state_info_buffer_.back()->pose;

    state_mean->twist.linear /= ((float)this->state_info_buffer_.size());
    state_mean->twist.lateral /= ((float)this->state_info_buffer_.size());
    state_mean->twist.angular /= ((float)this->state_info_buffer_.size());

    out = state_mean;

    //------------- PUBLISHING STATE -----------------------------
    ROS_DEBUG(
        "* %s. publishing new state mean for time (%lf)", getClassName(*this).c_str(), state_mean.getStamp().toSec());
    rtcus_conversions::StampedConversion<DynamicState2D>::StampedType stamped_out;
    rtcus_conversions::StampedConversion<DynamicState2D>::convert(out, stamped_out);
    this->processed_state_reading_pub_.publish(stamped_out);
    ROS_DEBUG(" * %s. Published", getClassName(*this).c_str());
  }
}

HighNoiseOdometryStatePort::HighNoiseOdometryStatePort() :
    configure_server_(ros::NodeHandle(this->component_node_.getNamespace() + "/high_noise_filter"))
{

}

void HighNoiseOdometryStatePort::init(std::string reference_frame)
{
  AdaptableStateCorrectionPortBase<DynamicState2D, nav_msgs::Odometry>::init(reference_frame);
  configure_server_.setCallback(boost::bind(&HighNoiseOdometryStatePort::configure_server_callback, this, _1));
  processed_state_reading_pub_ = this->component_node_.advertise
      < rtcus_conversions::StampedConversion<DynamicState2D>::StampedType > ("procesed_state_reading", 10);

}

HighNoiseOdometryStatePort::~HighNoiseOdometryStatePort()
{
}

}
}

