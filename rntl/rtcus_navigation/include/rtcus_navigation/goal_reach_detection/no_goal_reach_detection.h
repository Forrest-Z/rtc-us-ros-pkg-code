/*
 * default_goal_reach_detection.h
 *
 *  Created on: Sep 14, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef NO_GOAL_REACH_DETECTION_H_
#define NO_GOAL_REACH_DETECTION_H_

#include <rtcus_navigation/reached_goal_predicate.h>

namespace rtcus_navigation
{
namespace goal_reach_detection
{
using namespace rtcus_navigation;

template<typename StateType, typename GoalType>
  class NoReachedGoalPredicate : public ReachedGoalPredicate<StateType, GoalType>
  {

  public:
    NoReachedGoalPredicate()
    {

    }
    virtual ~NoReachedGoalPredicate()
    {
    }
    virtual void init()
    {

    }
    virtual void reset()
    {

    }

    virtual bool isGoalReached(const StateType& current_state, const GoalType& goal)
    {
      return false;
    }

  };
}
}

#endif /* DEFAULT_GOAL_REACH_DETECTION_H_ */
