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
#include <rtcus_compositions/impl/poses.h>

namespace rtcus_compositions
{

using namespace rtcus_nav_msgs;
using namespace mrpt::poses;

template<>
  void StateComposer::compose<CPose2D, CPose2D>(const mrpt::poses::CPose2D& state,
                                                const mrpt::poses::CPose2D& transform, mrpt::poses::CPose2D& dst)
  {
    dst = state + transform;
  }

template<>
  void StateComposer::compose<Pose2D, Pose2D>(const Pose2D& state, const Pose2D& transform, Pose2D& dst)
  {
    compose_pose_2d(state.x, state.y, state.phi, transform.x, transform.y, transform.phi, dst.x, dst.y, dst.phi);
  }

template<>
  void StateComposer::inverse_compose<Pose2D, Pose2D>(const Pose2D& state, const Pose2D& transform, Pose2D& dst,
                                                      const std::string& new_frame_name)
  {
    inverse_compose_pose_2d(state.x, state.y, state.phi, transform.x, transform.y, transform.phi, dst.x, dst.y,
                            dst.phi);
  }

template<>
  void StateComposer::compose<CPose3D, CPose3D>(const mrpt::poses::CPose3D& state,
                                                const mrpt::poses::CPose3D& transform, mrpt::poses::CPose3D& dst)
  {
    dst = state + transform;
  }

template<>
  void StateComposer::compose<geometry_msgs::Pose, geometry_msgs::Pose>(const geometry_msgs::Pose& state,
                                                                        const geometry_msgs::Pose& transform,
                                                                        geometry_msgs::Pose& dst)
  {
    mrpt::poses::CPose3D state_b;
    mrpt::poses::CPose3D transform_b;
    mrpt::poses::CPose3D dst_b;
    mrpt_bridge::poses::ros2mrpt(state, state_b);
    mrpt_bridge::poses::ros2mrpt(transform, transform_b);
    dst_b = state_b + transform_b;
    mrpt_bridge::poses::mrpt2ros(dst_b, dst);
  }

template<>
  void StateComposer::inverse_compose<geometry_msgs::Pose, geometry_msgs::Pose>(
      const geometry_msgs::Pose& state, const geometry_msgs::Pose& new_local_frame, geometry_msgs::Pose& dst,
      const std::string& new_frame_name)
  {
    mrpt::poses::CPose3D state_b;
    mrpt::poses::CPose3D new_local_frame_b;
    mrpt::poses::CPose3D dst_b;
    mrpt_bridge::poses::ros2mrpt(state, state_b);
    mrpt_bridge::poses::ros2mrpt(new_local_frame, new_local_frame_b);
    dst_b = state_b - new_local_frame_b;
    mrpt_bridge::poses::mrpt2ros(dst_b, dst);
  }
}
