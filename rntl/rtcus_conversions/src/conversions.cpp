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
#include <rtcus_nav_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <rtcus_nav_msgs/DynamicState2D.h>
#include <tf/tf.h>
#include <pcl/point_types.h>

namespace rtcus_conversions
{
using namespace rtcus_nav_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;

// ================== DERIVED CONVERSIONS ====================
template<typename Tdst>
  void aux_convert(const Tdst& src, Pose2D& dst)
  {
    Pose intermediate;
    Conversions::convert(src, intermediate);
    Conversions::convert(intermediate, dst);
  }

template<>
  void Conversions::convert<Pose2D, Pose>(const Pose2D& src, Pose& dst)
  {
    dst.position.x = src.x;
    dst.position.y = src.y;
    dst.position.z = 0;

    tf::Quaternion q = tf::createQuaternionFromYaw(src.phi);
    tf::quaternionTFToMsg(q, dst.orientation);
  }

template<>
  void Conversions::convert<pcl::PointXY, pcl::PointXY>(const pcl::PointXY& src, pcl::PointXY& dst)
  {
    dst = src;
  }
template<>
  void Conversions::convert<rtcus_nav_msgs::Pose2D, pcl::PointXY>(const rtcus_nav_msgs::Pose2D& src, pcl::PointXY& dst)
  {
    dst.x = src.x;
    dst.y = src.y;
  }

template<>
  void Conversions::convert<PoseStamped, Pose2D>(const PoseStamped& src, Pose2D& dst)
  {
    aux_convert(src, dst);
  }

//============== NOT ORTOGONAL CONVERSIONS =================
template<>
  void Conversions::convert<Pose2D, tf::Transform>(const Pose2D& src, tf::Transform& dst)
  {
    dst.setOrigin(tf::Vector3(src.x, src.y, 0.0));
    dst.setRotation(tf::createQuaternionFromYaw(src.phi));
  }

template<>
  void Conversions::convert<pcl::PointXY, tf::Transform>(const pcl::PointXY& src, tf::Transform& dst)
  {
    dst.setOrigin(tf::Vector3(src.x, src.y, 0.0));
    dst.setRotation(tf::Quaternion(0, 0, 0));
  }

template<>
  void Conversions::convert<pcl::PointXY, pcl::PointXYZ>(const pcl::PointXY& src, pcl::PointXYZ& dst)
  {
    dst.x = src.x;
    dst.y = src.y;
    dst.z = 0.0;
  }

template<>
  void Conversions::convert<Pose, Pose2D>(const Pose& src, Pose2D& dst)
  {
    dst.x = src.position.x;
    dst.y = src.position.y;

    tf::Quaternion q;
    tf::quaternionMsgToTF(src.orientation, q);
    dst.phi = tf::getYaw(q);
  }
template<>
  void Conversions::convert<DynamicState2D, tf::Transform>(const DynamicState2D& src, tf::Transform& dst)
  {
    dst.setOrigin(tf::Vector3(src.pose.x, src.pose.y, 0.0));
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, src.pose.phi);
    dst.setRotation(q);
  }

template<>
  void Conversions::convert<Twist2D, Twist>(const Twist2D& src, Twist& dst)
  {
    dst.linear.x = src.linear;
    dst.linear.y = src.lateral;
    dst.angular.z = src.angular;
  }

template<>
  void Conversions::convert<Twist, Twist2D>(const Twist& src, Twist2D& dst)
  {
    dst.linear = src.linear.x;
    dst.lateral = src.linear.y;
    dst.angular = src.angular.z;
  }

// ================ ORTOGONAL ROS MSGS =======================

template<>
  void Conversions::convert<PoseStamped, Pose>(const PoseStamped& src, Pose& dst)
  {
    dst = src.pose;
  }
}

