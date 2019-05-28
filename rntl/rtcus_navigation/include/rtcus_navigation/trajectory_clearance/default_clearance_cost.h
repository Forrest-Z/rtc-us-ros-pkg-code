/*
 * default_clearance_cost.h
 *
 *  Created on: May 18, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef DEFAULT_CLEARANCE_COST_H_
#define DEFAULT_CLEARANCE_COST_H_
#include <pcl/point_types.h>

namespace rtcus_navigation
{
namespace trajectory_clearance
{
class DefaultClearanceCost
{
public:
  DefaultClearanceCost() :
      collision(false), linear_collision_distance_(std::numeric_limits<double>::quiet_NaN()), angular_collision_distance_(
          std::numeric_limits<double>::quiet_NaN())
  {

  }
  bool collision;
  double linear_collision_distance_;
  double angular_collision_distance_;
  double time_to_collision_;
  pcl::PointXY collision_point_;
};
}
}

#endif /* DEFAULT_CLEARANCE_COST_H_ */
