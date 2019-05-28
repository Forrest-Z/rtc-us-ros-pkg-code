/*
 * conversions.cpp
 *
 *  Created on: Jun 18, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_conversions/conversions.h>
#include <geometry_msgs/Twist.h>
#include <rtcus_nav_msgs/Twist2D.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <rtcus_nav_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <rtcus_nav_msgs/DynamicState2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

namespace rtcus_conversions
{
using namespace geometry_msgs;

template<>
  void Conversions::convert<geometry_msgs::PoseStamped, pcl::PointXY>(const PoseStamped& src, pcl::PointXY& dst)
  {
    dst.x = src.pose.position.x;
    dst.y = src.pose.position.y;
  }

template<>
  void Conversions::convert<geometry_msgs::Pose, pcl::PointXY>(const Pose& src, pcl::PointXY& dst)
  {
    dst.x = src.position.x;
    dst.y = src.position.y;
  }

template<>
  void Conversions::convert<pcl::PointXY, geometry_msgs::Pose>(const pcl::PointXY& src, Pose& dst)
  {
    dst.position.x = src.x;
    dst.position.y = src.y;
    dst.position.z = 0;
  }

//for example for trajectory painting
template<>
  void Conversions::convert(const rtcus_nav_msgs::DynamicState2D& input, geometry_msgs::Point& output)
  {
    output.x = input.pose.x;
    output.y = input.pose.y;
    output.z = 0;
  }

template<>
  void Conversions::convert(const rtcus_nav_msgs::Pose2D& input, geometry_msgs::Point& output)
  {
    output.x = input.x;
    output.y = input.y;
    output.z = 0;
  }

//STATE MSG ADAPTER
template<>
  void Conversions::convert(const nav_msgs::Odometry& input, rtcus_nav_msgs::DynamicState2D& output)
  {
    rtcus_nav_msgs::DynamicState2D ret;
    output.pose.x = input.pose.pose.position.x;
    output.pose.y = input.pose.pose.position.y;

    //phi==yaw in 2D
    tf::Quaternion quat;
    tf::quaternionMsgToTF(input.pose.pose.orientation, quat);
    output.pose.phi = tf::getYaw(quat);
    output.twist.linear = input.twist.twist.linear.x;
    output.twist.angular = input.twist.twist.angular.z;
  }
;
}

