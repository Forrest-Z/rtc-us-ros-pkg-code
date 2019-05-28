/*
 * common.h
 *
 *  Created on: Apr 10, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef COMMON_RTCUS_NAVIGATION_H_
#define COMMON_RTCUS_NAVIGATION_H_

#include <list>
#include <queue>
#include <vector>

#include <rtcus_conversions/conversions.h>
#include <rtcus_compositions/state_composer.h>
#include <rtcus_stamp/is_headed_msg.h>
#include <rtcus_stamp/stamped.h>
#include <rtcus_motion_models/motion_models.h>

#include <rtcus_nav_msgs/Twist2D.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace rtcus_navigation
{
typedef pcl::PointXY PointXY;

}

inline bool ARE_SAME_FRAMES(const std::string& x, const std::string& y)
{
  return x == y || (x.size() > 0 && x[0] == '/' && x.substr(1) == y)
      || (y.size() > 0 && y[0] == '/' && y.substr(1) == x);
}

inline bool ARE_DISTINCT_FRAMES(const std::string& x, const std::string& y)
{

  return !ARE_SAME_FRAMES(x, y);
}
#define STRINGFY(a) #a
#define STRINGFY_CLASS_NAME(name, a) #name "_" #a
#define STRINGFY_CLASS_NAME_2(name, a,b) #name "_" #a "_" #b
#define STRINGFY_CLASS_NAME_3(name, a,b,c) #name "_" #a "_" #b "_" #c
#define STRINGFY_CLASS_NAME_4(name, a,b,c,d) #name "_" #a "_" #b "_" #c "_" #d

template<typename T>
  inline std::string getClassName(const T& object)
  {
    return boost::units::detail::demangle(typeid(object).name());
  }

#endif /* COMMON_H_ */
