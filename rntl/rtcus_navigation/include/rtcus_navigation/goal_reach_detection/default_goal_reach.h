/*
 * dwa_goal_reach_predicate.h
 *
 *  Created on: Sep 17, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef DWA_GOAL_REACH_PREDICATE_H_
#define DWA_GOAL_REACH_PREDICATE_H_

#include <rtcus_navigation/goal_reach_detection/default_goal_reach_detection_base.h>
#include <rtcus_nav_msgs/DynamicState2D.h>
#include <pcl/point_types.h>

namespace rtcus_navigation
{
namespace goal_reach_detection
{
using namespace rtcus_nav_msgs;

class DefaultGoalReachPredicate : public rtcus_navigation::goal_reach_detection::ReachedGoalPredicateBase<
    DynamicState2D, pcl::PointXY>
{
public:
  virtual ~DefaultGoalReachPredicate();
  virtual float distance(const rtcus_nav_msgs::DynamicState2D & state, const pcl::PointXY& goal);
};
}
}

#endif /* DWA_GOAL_REACH_PREDICATE_H_ */
