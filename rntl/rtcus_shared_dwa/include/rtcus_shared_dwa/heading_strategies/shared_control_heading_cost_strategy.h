/*
 * trajectory_distance_heading_cost_strategy.h
 *
 *  Created on: Oct 25, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef SHARED_CONTROL_HEADING_COST_STRATEGY_H_
#define SHARED_CONTROL_HEADING_COST_STRATEGY_H_

#include <rtcus_dwa/heading_cost_strategies.h>

namespace rtcus_shared_dwa
{
namespace heading_strategies
{

using namespace rtcus_dwa;

/**
 * \brief The goal represent the desired action in the whole action space (not just the dynamic window).
 * The best action is considered that such the euclidean distance in this the following domain is minimized:
 * The goal Twist2D is an action linear [-1 to 1] angular [0 to 1].
 * */
class SharedControlHeadingCostStrategy : public HeadingCostStrategy<rtcus_nav_msgs::Twist2D, rtcus_nav_msgs::Twist2D,
    rtcus_nav_msgs::Twist2D>
{
protected:

  rtcus_nav_msgs::Twist2D normalized_goal_;

public:
  SharedControlHeadingCostStrategy();
  virtual ~SharedControlHeadingCostStrategy();

  virtual void init(const DwaConfig& config, const Twist2D& state, const DWAMotionModel& motion_model,
                    const rtcus_nav_msgs::Twist2D& goal);

  virtual t_float computeCost(const rtcus_dwa::CommandCost<rtcus_nav_msgs::Twist2D>& action);

};
}
}

#endif /* TRAJECTORY_DISTANCE_HEADING_COST_STRATEGY_H_ */
