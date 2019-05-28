/*
 *
 *  Created on: 18/12/2011
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 */

#include <ros/ros.h>
#include <ros/duration.h>
#include <rtcus_navigation/navigation_node.h>
#include <rtcus_navigation/impl/navigation_node.h>

#include <rtcus_navigation/world_perception_ports/point_cloud.h>
#include <rtcus_navigation/world_perception_ports/laser_scan.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <rtcus_kinodynamic_description/NoKinoDynamicsConfig.h>
#include <rtcus_robot_shapes/circular_robot.h>

using namespace geometry_msgs;
using namespace nav_msgs;
using namespace rtcus_navigation;
using namespace rtcus_navigation::world_perception_ports;
using namespace rtcus_kinodynamic_description;
using namespace rtcus_robot_shapes;

template class NavigationNode<Pose, pcl::PointCloud<pcl::PointXYZ>, Twist, Pose, CircularRobot, NoKinoDynamicsConfig,
    ROSTimeModel> ;

namespace rtcus_navigation
{
const std::string& getComponentTypeName(NavComponentType type)
{
  return __navComponentsNames[type];
}
}
