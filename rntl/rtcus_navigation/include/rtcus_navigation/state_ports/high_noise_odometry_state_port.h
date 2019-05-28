/*
 * high_noise_odometry_state_port.h
 *
 *  Created on: Jan 17, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef HIGH_NOISE_ODOMETRY_STATE_PORT_H_
#define HIGH_NOISE_ODOMETRY_STATE_PORT_H_

#include <rtcus_navigation/state_ports/adaptable_state_correction_port.h>
#include <nav_msgs/Odometry.h>
#include <rtcus_navigation/HighNoiseOdometryStatePortConfig.h>
#include <dynamic_reconfigure/server.h>
#include <rtcus_conversions/conversions_stamped.h>

namespace rtcus_navigation
{
namespace state_ports
{
/**
 * \brief converts the odometry input data in a smoother trajectory. It can make introduce a delay so it is desirable to be
 * used in conjuntion with the PTC method.
 * */
class HighNoiseOdometryStatePort : public AdaptableStateCorrectionPortBase<DynamicState2D, nav_msgs::Odometry>
{
protected:
  std::list<StampedData<DynamicState2D> > state_info_buffer_;
  boost::mutex m_mutex;
  ros::Publisher processed_state_reading_pub_;

  HighNoiseOdometryStatePortConfig config_;
  dynamic_reconfigure::Server<rtcus_navigation::HighNoiseOdometryStatePortConfig> configure_server_;
  const HighNoiseOdometryStatePortConfig& configure_server_callback(HighNoiseOdometryStatePortConfig &config);

  virtual void fromInputRosMsgToInternalState(const nav_msgs::Odometry& msg, DynamicState2D& output_state,
                                              ros::Time& output_stamp, std::string& frame_id);

  void compute_mean_and_pop(const StampedData<DynamicState2D>& newreading, StampedData<DynamicState2D>& out);

public:

  HighNoiseOdometryStatePort();
  virtual void init(std::string reference_frame);
  virtual ~HighNoiseOdometryStatePort();
};
}
}

#endif /* HIGH_NOISE_ODOMETRY_STATE_PORT_H_ */
