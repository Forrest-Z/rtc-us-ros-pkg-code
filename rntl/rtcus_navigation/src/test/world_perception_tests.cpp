/*
 * world_perception_tests.cpp
 *
 *  Created on: Apr 10, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <mrpt_bridge/pose_conversions.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/duration.h>
#include <rtcus_stamp/stamped.h>

#include <rtcus_navigation/world_perception_ports/point_cloud.h>
#include <rtcus_navigation/world_perception_ports/laser_scan.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
using namespace rtcus_navigation;
using namespace rtcus_navigation::world_perception_ports;

void event_update(WorldPerceptionPort<pcl::PointCloud<pcl::PointXYZ> >& sender)
{
  ROS_INFO("[event] obstacles received ... ");
}

TEST(WorldPerception, BasicInitialization)
{
  ros::NodeHandle test;

  rtcus_navigation::world_perception_ports::PointCloudObstacles<pcl::PointXYZ> obstacles_perception;
  PointCloudXYZ::Ptr msg(new PointCloudXYZ);
  obstacles_perception.init();

  ros::Publisher test_pub = test.advertise<PointCloudXYZ>("/cloud", 1);

  //we set the sensor frame name to avoid coherence check errors
  msg->header.frame_id = obstacles_perception.getSensorFrameName();
  msg->header.stamp = ros::Time::now();
  msg->height = msg->width = 1;
  msg->height = msg->height = 1;
  msg->points.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
  test_pub.publish(msg);
  obstacles_perception.onWorldUpdate.connect(event_update);

  for (int i = 0; i < 10; i++)
  {
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  rtcus_stamp::Stamped < PointCloudXYZ > lastupdate;
  boost::shared_ptr<boost::mutex> mutex = obstacles_perception.getLastUpdateWorld(
      lastupdate);

  {
    boost::mutex::scoped_lock(mutex);
    ASSERT_EQ(lastupdate->points.size(), msg->points.size());
    ASSERT_EQ(lastupdate.getStamp(), msg->header.stamp);
    ASSERT_EQ(lastupdate.getFrameId(), obstacles_perception.getMobileBaseFrameName());
  }
}

TEST(WorldPerception, LaserScanPointCloud)
{
  rtcus_navigation::world_perception_ports::PointCloudObstaclesLaserScan<pcl::PointXY> obstacles_perception;
  obstacles_perception.init();

  sensor_msgs::LaserScan scan;
  scan.range_max = 10;
  scan.range_min = -10;
  scan.ranges.push_back(0);
  scan.ranges.push_back(20);
  scan.ranges.push_back(10);

  obstacles_perception.pushObstacles(scan);

}
