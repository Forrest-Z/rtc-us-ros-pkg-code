/*
 * point_cloud_xyz.h
 *
 *  Created on: Apr 10, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef POINT_CLOUD_XYZ_H_
#define POINT_CLOUD_XYZ_H_
#include <rtcus_navigation/world_perception_ports/point_cloud_perception_base.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>

namespace rtcus_navigation
{
namespace world_perception_ports
{
template<typename PointCloudType>
  class PointCloudObstacles : public PointCloudObstaclesBase<PointCloudType>
  {

  protected:

    virtual void init_subscriber()
    {
      ros::NodeHandle& nh = this->node_;
      //EFFICIENT WAY
      //this->scan_sub_ = nh.subscribe<pcl::PointCloud<PointCloudType> >(
      //    DEFAULT_POINT_CLOUD_TOPIC_NAME, 10, &PointCloudObstacles<PointCloudType>::pointCloudCallback, this);

      //GENERALISTIC WAY
      this->scan_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
          DEFAULT_POINT_CLOUD_TOPIC_NAME, 10, &PointCloudObstacles<PointCloudType>::pointCloud2Callback, this);

    }

    void pointCloud2Callback(const boost::shared_ptr<const sensor_msgs::PointCloud2> & obstacles_msg)
    {
      pcl::PointCloud<PointCloudType> cloud_pcl;
      pcl::fromROSMsg(*obstacles_msg, cloud_pcl);
      this->pushObstacles(cloud_pcl);
    }

    void pointCloudCallback(const boost::shared_ptr<const pcl::PointCloud<PointCloudType> > & obstacles_msg)
    {
      this->pushObstacles(*obstacles_msg);
    }
  public:
    PointCloudObstacles() :
        PointCloudObstaclesBase<PointCloudType>::PointCloudObstaclesBase()
    {

    }
    virtual ~PointCloudObstacles()
    {
    }
  };

template class PointCloudObstacles<pcl::PointXY> ;
template class PointCloudObstacles<pcl::PointXYZ> ;
}
}

#endif /* POINT_CLOUD_H_ */
