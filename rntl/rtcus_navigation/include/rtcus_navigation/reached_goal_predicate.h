/*
 *  Created on: 18/12/2011
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef REACHED_GOAL_PREDICATE_H_
#define REACHED_GOAL_PREDICATE_H_
#include <rtcus_navigation/navigation_component.h>

namespace rtcus_navigation
{

/**
 * \brief This class defines if the goal has been reached given the current state. This is typically used
 * by Navigation Nodes or Navigation Planners to stop the execution.
 * */
template<typename StateType, typename GoalType>
  class ReachedGoalPredicate : public NavigationNodeComponent
  {

  public:
    ReachedGoalPredicate() :
        NavigationNodeComponent(tGoalReachedDetection)
    {

    }

    virtual ~ReachedGoalPredicate()
    {
    }

    virtual void reset()=0;
    virtual void init()=0;

    /**
     * @brief  Check if the goal pose has been achieved by the local planner
     * @return True if achieved, false otherwise
     */
    virtual bool isGoalReached(const StateType& current_state, const GoalType& goal) =0;

  };
}
;
#endif /* LOCALPLANNERBASE_H_ */
