/*
 * state_composer.cpp
 *
 *  Created on: Apr 19, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_compositions/state_composer.h>

#include <geometry_msgs/Pose.h>
#include <mrpt/poses.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <mrpt_bridge/pose_conversions.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h>
#include <rtcus_nav_msgs/DynamicState2D.h>
#include <rtcus_nav_msgs/Twist2D.h>
#include <rtcus_nav_msgs/Pose2D.h>
#include <rtcus_compositions/impl/points.h>

namespace rtcus_compositions
{

using namespace rtcus_nav_msgs;
using namespace mrpt::poses;

template<>
  void StateComposer::inverse_compose<pcl::PointXY, mrpt::poses::CPose2D>(const pcl::PointXY& state,
                                                                          const mrpt::poses::CPose2D& local_frame,
                                                                          pcl::PointXY& dst,
                                                                          const std::string& new_frame_name)
  {
    inverse_compose_point_2D<float>(state.x, state.y, local_frame.x(), local_frame.y(), local_frame.phi(), dst.x,
                                    dst.y);
  }

template<>
  void StateComposer::inverse_compose<pcl::PointXY, rtcus_nav_msgs::Pose2D>(const pcl::PointXY& state,
                                                                            const rtcus_nav_msgs::Pose2D& local_frame,
                                                                            pcl::PointXY& dst,
                                                                            const std::string& new_frame_name)
  {
    inverse_compose_point_2D<float>(state.x, state.y, local_frame.x, local_frame.y, local_frame.phi, dst.x, dst.y);
  }

template<>
  void StateComposer::compose<pcl::PointXY, rtcus_nav_msgs::Pose2D>(const pcl::PointXY& state,
                                                                    const rtcus_nav_msgs::Pose2D& transform,
                                                                    pcl::PointXY& dst)
  {

    compose_point_2d<float>(state.x, state.y, transform.x, transform.y, transform.phi, dst.x, dst.y);
  }
}
