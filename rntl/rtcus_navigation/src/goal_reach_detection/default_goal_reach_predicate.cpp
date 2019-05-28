/*
 * default_goal_reach_predicate.cpp
 *
 *  Created on: Nov 23, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_navigation/goal_reach_detection/default_goal_reach.h>

namespace rtcus_navigation
{
namespace goal_reach_detection
{
DefaultGoalReachPredicate::~DefaultGoalReachPredicate()
{
}

float DefaultGoalReachPredicate::distance(const rtcus_nav_msgs::DynamicState2D & state, const pcl::PointXY& goal)
{
  float xdiff = state.pose.x - goal.x;
  float ydiff = state.pose.y - goal.y;
  double distance = sqrt(xdiff * xdiff + ydiff * ydiff);
  return distance;
}
}
}
