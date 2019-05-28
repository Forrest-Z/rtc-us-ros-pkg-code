/*
 * inverse_composition_point_cloud.cpp
 *
 *  Created on: Jun 18, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_compositions/state_composer.h>
#include <rtcus_compositions/impl/point_clouds_common.h>

#include <rtcus_nav_msgs/DynamicState2D.h>
#include <rtcus_nav_msgs/Pose2D.h>

using namespace rtcus_nav_msgs;

//==============================================
inline void aux_pose2d_to_transform(const float x, const float y, const float phi, tf::Transform& transf)
{
  transf.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion rot;
  rot.setRPY(0.0, 0.0, phi);
  transf.setRotation(rot);
}
//==================================== COMPOSITIONS =================================================================
namespace rtcus_compositions
{

//PointXY + transf
template<>
  void StateComposer::compose<pcl::PointCloud<pcl::PointXY>, tf::Transform>(
      const pcl::PointCloud<pcl::PointXY>& input_cloud, const tf::Transform& transf,
      pcl::PointCloud<pcl::PointXY>& resulting_cloud)
  {

    aux_compose_pointCloud_T(input_cloud, transf, resulting_cloud);
  }

//PointXY + Pose2D
template<>
  void StateComposer::compose<pcl::PointCloud<pcl::PointXY>, rtcus_nav_msgs::Pose2D>(
      const pcl::PointCloud<pcl::PointXY>& input_cloud, const rtcus_nav_msgs::Pose2D& transf,
      pcl::PointCloud<pcl::PointXY>& resulting_cloud)
  {
    tf::Transform t;
    aux_pose2d_to_transform(transf.x, transf.y, transf.phi, t);
    aux_compose_pointCloud_T(input_cloud, t, resulting_cloud);
  }

//PointXY + Pose
template<>
  void StateComposer::compose<pcl::PointCloud<pcl::PointXY>, geometry_msgs::Pose>(
      const pcl::PointCloud<pcl::PointXY>& input_cloud, const geometry_msgs::Pose& transf,
      pcl::PointCloud<pcl::PointXY>& resulting_cloud)
  {
    tf::Transform transf2;
    tf::poseMsgToTF(transf, transf2);
    aux_compose_pointCloud_T(input_cloud, transf2, resulting_cloud);
  }

// ============================== INVERSE COMPOSITIONS ===============================================================

//Point Vectors
//PointXY + Pose2D

// PointXY  - POSE
template<>
  void StateComposer::inverse_compose<pcl::PointCloud<pcl::PointXY>, geometry_msgs::Pose>(
      const pcl::PointCloud<pcl::PointXY>& input_cloud, const geometry_msgs::Pose& new_local_frame,
      pcl::PointCloud<pcl::PointXY>& resulting_cloud, const std::string& local_frame_name)
  {
    tf::Transform transf;
    tf::poseMsgToTF(new_local_frame, transf);
    if (local_frame_name != ANONYMOUS_LOCAL_FRAME_NAME)
      resulting_cloud.header.frame_id = local_frame_name;
    transform_point_cloud_aux(input_cloud, transf.inverse(), resulting_cloud, local_frame_name);
  }

// PointXY  - POSE2D
template<>
  void StateComposer::inverse_compose<pcl::PointCloud<pcl::PointXY>, rtcus_nav_msgs::Pose2D>(
      const pcl::PointCloud<pcl::PointXY>& input_cloud, const rtcus_nav_msgs::Pose2D& new_local_frame,
      pcl::PointCloud<pcl::PointXY>& resulting_cloud, const std::string& local_frame_name)
  {

    tf::Transform transf;
    aux_pose2d_to_transform(new_local_frame.x, new_local_frame.y, new_local_frame.phi, transf);

    if (local_frame_name != ANONYMOUS_LOCAL_FRAME_NAME)
      resulting_cloud.header.frame_id = local_frame_name;
    transform_point_cloud_aux(input_cloud, transf.inverse(), resulting_cloud, local_frame_name);

    /*tf::Transform transf;
     aux_pose2d_to_transform(0, 0, new_local_frame.phi, transf);

     if (local_frame_name != ANONYMOUS_LOCAL_FRAME_NAME)
     resulting_cloud.header.frame_id = local_frame_name;

     transform_point_cloud_aux(input_cloud, transf.inverse(), resulting_cloud, local_frame_name);
     aux_pose2d_to_transform(new_local_frame.x, new_local_frame.y, 0, transf);
     transform_point_cloud_aux(resulting_cloud, transf.inverse(), resulting_cloud, local_frame_name);*/

  }

}
