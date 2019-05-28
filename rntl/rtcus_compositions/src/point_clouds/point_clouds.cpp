/*
 * inverse_composition_point_cloud.cpp
 *
 *  Created on: Jun 18, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_compositions/state_composer.h>
#include <rtcus_compositions/impl/point_clouds_common.h>

namespace rtcus_compositions
{
//==================================== COMPOSITIONS =================================================================
// WE NEED HERE A BIT OF METAPROGRAMMING
//PointXYZ + tranf
template<>
  void StateComposer::compose<pcl::PointCloud<pcl::PointXYZ>, tf::Transform>(
      const pcl::PointCloud<pcl::PointXYZ>& input_cloud, const tf::Transform& transf,
      pcl::PointCloud<pcl::PointXYZ>& resulting_cloud)
  {

    aux_compose_pointCloud_T(input_cloud, transf, resulting_cloud);
  }

//PointXYZ + Pose
template<>
  void StateComposer::compose<pcl::PointCloud<pcl::PointXYZ>, geometry_msgs::Pose>(
      const pcl::PointCloud<pcl::PointXYZ>& input_cloud, const geometry_msgs::Pose& transf,
      pcl::PointCloud<pcl::PointXYZ>& resulting_cloud)
  {
    tf::Transform transf2;
    tf::poseMsgToTF(transf, transf2);

    aux_compose_pointCloud_T(input_cloud, transf2, resulting_cloud);
  }

// ============================== INVERSE COMPOSITIONS ===============================================================
// PointXYZ  - POSE
template<>
  void StateComposer::inverse_compose<pcl::PointCloud<pcl::PointXYZ>, geometry_msgs::Pose>(
      const pcl::PointCloud<pcl::PointXYZ>& input_cloud, const geometry_msgs::Pose& new_local_frame,
      pcl::PointCloud<pcl::PointXYZ>& resulting_cloud, const std::string& local_frame_name)
  {
    tf::Transform transf;
    tf::poseMsgToTF(new_local_frame, transf);
    transform_point_cloud_aux(input_cloud, transf.inverse(), resulting_cloud, local_frame_name);
  }

//FROM TRANSFORM
template<>
  void StateComposer::inverse_compose<pcl::PointCloud<pcl::PointXYZ>, tf::Transform>(
      const pcl::PointCloud<pcl::PointXYZ>& input_cloud, const tf::Transform& new_local_frame,
      pcl::PointCloud<pcl::PointXYZ>& resulting_cloud, const std::string& local_frame_name)
  {
    transform_point_cloud_aux(input_cloud, new_local_frame.inverse(), resulting_cloud, local_frame_name);
  }

}
