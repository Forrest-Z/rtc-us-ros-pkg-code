/*
 * sharec_control_acceleration_heading_strategy.h
 *
 *  Created on: Feb 13, 2013
 *      Author: root
 */

#ifndef SHAREC_CONTROL_ACCELERATION_HEADING_STRATEGY_H_
#define SHAREC_CONTROL_ACCELERATION_HEADING_STRATEGY_H_

#include <rtcus_shared_dwa/heading_strategies/shared_control_heading_cost_strategy.h>

namespace rtcus_shared_dwa
{
namespace heading_strategies
{
using namespace rtcus_dwa;
/**
 * \brief The goal represent the desired action in the current dynamic window. The goal Twist2D is an action linear [-1 to 1] angular [-1 to 1].
 * */
class SharedControlAccelerationHeadingCostStrategy : public SharedControlHeadingCostStrategy
{
protected:
  t_float normalization;
public:
  SharedControlAccelerationHeadingCostStrategy();
  virtual void init(const DwaConfig& config, const Twist2D& state, const DWAMotionModel& motion_model,
                    const rtcus_nav_msgs::Twist2D& goal);

  virtual t_float computeCost(const DwaCommandCost& action);
};
}
}

#endif /* SHAREC_CONTROL_ACCELERATION_HEADING_STRATEGY_H_ */
