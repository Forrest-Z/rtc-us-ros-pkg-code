/*
 * trajectory_distance_heading_cost_strategy.h
 *
 *  Created on: Oct 25, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef TRAJECTORY_DISTANCE_HEADING_COST_STRATEGY_H_
#define TRAJECTORY_DISTANCE_HEADING_COST_STRATEGY_H_

#include <rtcus_dwa/heading_cost_strategies.h>
#include <rtcus_dwa/heading_strategies/default_dwa_heading_cost_strategy.h>

namespace rtcus_dwa
{
namespace heading_strategies
{

class TragectoryGoalIntersectionHeadingCostStrategy : public rtcus_dwa::HeadingCostStrategy<pcl::PointXY, Twist2D,
    Twist2D>
{
public:
  virtual t_float computeCost(const CommandCost<Twist2D>& action);
  virtual ~TragectoryGoalIntersectionHeadingCostStrategy()
  {
  }
  ;
};
}

}

#endif /* TRAJECTORY_DISTANCE_HEADING_COST_STRATEGY_H_ */
