/*
 * point_cloud_conversions.cpp
 *
 *  Created on: Dec 5, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <mrpt/slam/CSimplePointsMap.h>
#include <mrpt_bridge/point_cloud.h>
#include <rtcus_conversions/conversions.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace rtcus_conversions
{
using namespace sensor_msgs;


template<>
  void Conversions::convert<LaserScan, PointCloud2>(const LaserScan& src, PointCloud2& dst)
  {
    static laser_geometry::LaserProjection laser_projection;
    laser_projection.projectLaser(src, dst);
  }

template<>
  void Conversions::convert<pcl::PointCloud<pcl::PointXY>, mrpt::slam::CSimplePointsMap>(
      const pcl::PointCloud<pcl::PointXY> & src, mrpt::slam::CSimplePointsMap& dst)
  {
    const size_t N = src.points.size();
    dst.clear();
    dst.reserve(N);
    for (size_t i = 0; i < N; ++i)
      dst.insertPointFast(src.points[i].x, src.points[i].y, 0.0);
  }
}
