/*
 * laser_scan.h
 *
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef LASER_SCAN_H_
#define LASER_SCAN_H_

#include <rtcus_navigation/world_perception_ports/point_cloud_perception_base.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <rtcus_conversions/conversions.h>
#include <pcl/ros/conversions.h>

namespace rtcus_navigation
{
namespace world_perception_ports
{

template<typename PointCloudType>
  class PointCloudObstaclesLaserScan : public PointCloudObstaclesBase<PointCloudType>
  {

#define DEFAULT_LASER_SCAN_TOPIC_NAME "scan"

  protected:

    virtual void init_subscriber()
    {
      ros::NodeHandle& nh = this->node_;
      this->scan_sub_ = nh.subscribe<sensor_msgs::LaserScan>(
          DEFAULT_LASER_SCAN_TOPIC_NAME, 10, &PointCloudObstaclesLaserScan<PointCloudType>::scanCallback, this);
    }

  public:
    PointCloudObstaclesLaserScan() :
        PointCloudObstaclesBase<PointCloudType>::PointCloudObstaclesBase()
    {

    }
    virtual ~PointCloudObstaclesLaserScan()
    {

    }
//laser input
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
      pushObstacles(*scan);
    }

    void pushObstacles(const sensor_msgs::LaserScan& scan)
    {
      sensor_msgs::PointCloud2 temp_pc;
      rtcus_conversions::Conversions::convert(scan, temp_pc);

      pcl::PointCloud<PointCloudType> temp_pc2;
      pcl::fromROSMsg(temp_pc, temp_pc2);
      PointCloudObstaclesBase < PointCloudType > ::pushObstacles(temp_pc2);
    }
  }
  ;
template class PointCloudObstaclesLaserScan<pcl::PointXY> ;
template class PointCloudObstaclesLaserScan<pcl::PointXYZ> ;
}
}
#endif /* LASER_SCAN_H_ */
