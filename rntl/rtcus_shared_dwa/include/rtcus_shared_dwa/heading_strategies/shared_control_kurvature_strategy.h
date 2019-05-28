/*
 * shared_control_kurvature_strategy.h
 *
 *  Created on: Feb 13, 2013
 *  Created on: Oct 25, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef SHARED_CONTROL_KURVATURE_STRATEGY_H_
#define SHARED_CONTROL_KURVATURE_STRATEGY_H_
#include <rtcus_shared_dwa/heading_strategies/shared_control_heading_cost_strategy.h>

namespace rtcus_shared_dwa
{
namespace heading_strategies
{

using namespace rtcus_dwa;

/**
 * \brief The goal represent the desired action in the whole action space (not just the dynamic window).
 * The best action is that which have the same curvature of the goal
 * The goal Twist2D is an action linear [-1 to 1] angular [0 to 1].
 * */
class SharedControlKurvatureHeadingCostStrategy : public SharedControlHeadingCostStrategy
{
protected:
  t_float goal_alpha_;
  t_float goal_magnitude_;
public:
  virtual void init(const DwaConfig& config, const Twist2D& state, const DWAMotionModel& motion_model,
                    const rtcus_nav_msgs::Twist2D& goal);
  SharedControlKurvatureHeadingCostStrategy();
  virtual t_float computeCost(const rtcus_dwa::CommandCost<rtcus_nav_msgs::Twist2D>& action);
};
}
}

#endif /* SHARED_CONTROL_KURVATURE_STRATEGY_H_ */
