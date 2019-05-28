/*
 * adaptable_conversion_goal_port.h
 *
 *  Created on: Oct 25, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef ADAPTABLE_CONVERSION_GOAL_PORT_H_
#define ADAPTABLE_CONVERSION_GOAL_PORT_H_

#include <rtcus_navigation/goal_ports/adaptable_goal_port.h>

namespace rtcus_navigation
{
namespace goal_ports
{
template<typename GoalType, typename RosMsgGoal>
  class AdaptableConversionGoalPort : public AdaptableGoalPort<GoalType, RosMsgGoal>
  {
  public:
    virtual ~AdaptableConversionGoalPort()
    {
    }
  protected:
    virtual void convert_msg(const RosMsgGoal& goal_pose_msg, GoalType& goal)
    {
      rtcus_conversions::Conversions::convert(goal_pose_msg, goal);
    }
  }
  ;
}
}

#endif /* ADAPTABLE_CONVERSION_GOAL_PORT_H_ */
