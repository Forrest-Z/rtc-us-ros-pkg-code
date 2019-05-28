/*
 * incomplete_dynamic_state_compositions.cpp
 *
 *  Created on: Nov 23, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */
#include <rtcus_compositions/state_composer.h>
#include <rtcus_compositions/impl/points.h>
#include <rtcus_nav_msgs/DynamicState2D.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace rtcus_compositions
{

using namespace rtcus_nav_msgs;
using namespace pcl;

template<>
  void StateComposer::compose<Pose2D, DynamicState2D>(const Pose2D& state, const DynamicState2D& transform, Pose2D& dst)
  {
    StateComposer::compose(state, transform.pose, dst);

  }
template<>
  void StateComposer::inverse_compose<PointXY, DynamicState2D>(const PointXY& goal, const DynamicState2D& transform,
                                                               PointXY& dst, const std::string& new_frame_name)
  {
    rtcus_compositions::inverse_compose_point_2D<float>(goal.x, goal.y, transform.pose.x, transform.pose.y,
                                                        transform.pose.phi, dst.x, dst.y);
  }

template<>
  void StateComposer::compose<pcl::PointCloud<pcl::PointXY>, rtcus_nav_msgs::DynamicState2D>(
      pcl::PointCloud<pcl::PointXY> const& src, DynamicState2D const& transf, pcl::PointCloud<pcl::PointXY>& dst)
  {

    StateComposer::compose(src, transf.pose, dst);
  }

template<>
  void StateComposer::inverse_compose<pcl::PointCloud<pcl::PointXY>, rtcus_nav_msgs::DynamicState2D>(
      const pcl::PointCloud<pcl::PointXY>& input_cloud, const rtcus_nav_msgs::DynamicState2D& new_local_frame,
      pcl::PointCloud<pcl::PointXY>& resulting_cloud, const std::string& local_frame_name)
  {
    StateComposer::inverse_compose(input_cloud, new_local_frame.pose, resulting_cloud, local_frame_name);
  }
}

