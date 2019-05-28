/*
 * point_clouds_common.h
 *
 *  Created on: Jun 29, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef POINT_CLOUDS_COMMON_H_
#define POINT_CLOUDS_COMMON_H_

#include <rtcus_compositions/state_composer.h>
#include <geometry_msgs/Pose.h>
#include <mrpt/poses.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <mrpt_bridge/pose_conversions.h>
#include <tf/tf.h>

#include <sensor_msgs/PointCloud2.h>
#include <rtcus_nav_msgs/Twist2D.h>

namespace rtcus_compositions
{

//inspired on http://www.ros.org/doc/api/tf/html/classtf_1_1TransformListener.html#8b9cb24fdcdf596aca647327a666f502
void transform_point_cloud_aux(sensor_msgs::PointCloud2 const& input_cloud, tf::Transform const& transf,
                               sensor_msgs::PointCloud2& resulting_cloud);

template<typename T>
  inline void transform_point_cloud_aux(const pcl::PointCloud<T>& input_cloud, const tf::Transform& new_local_frame,
                                        pcl::PointCloud<T>& resulting_cloud, const std::string& local_frame_name)
  {
    sensor_msgs::PointCloud2 input, output;
    pcl::toROSMsg(input_cloud, input);

    if (local_frame_name != ANONYMOUS_LOCAL_FRAME_NAME)
      resulting_cloud.header.frame_id = local_frame_name;

    transform_point_cloud_aux(input, new_local_frame, output);
    pcl::fromROSMsg(output, resulting_cloud);
  }

//GENERIC COMPOSITION FOR POINTCLOUD<T>
template<typename T>
  void aux_compose_pointCloud_T(const pcl::PointCloud<T>& input_cloud, const tf::Transform& transf,
                                pcl::PointCloud<T>& resulting_cloud)
  {
    sensor_msgs::PointCloud2 input, output;
    assert(input_cloud.height*input_cloud.width==input_cloud.points.size());

    pcl::toROSMsg(input_cloud, input);
    transform_point_cloud_aux(input, transf, output);
    pcl::fromROSMsg(output, resulting_cloud);
  }
}

#endif /* POINT_CLOUDS_COMMON_H_ */
