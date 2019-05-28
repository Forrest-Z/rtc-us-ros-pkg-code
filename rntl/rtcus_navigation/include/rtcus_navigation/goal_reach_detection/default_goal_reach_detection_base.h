/*
 * default_goal_reach_detection.h
 *
 *  Created on: Sep 14, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef DEFAULT_GOAL_REACH_DETECTION_H_
#define DEFAULT_GOAL_REACH_DETECTION_H_

#include <rtcus_navigation/reached_goal_predicate.h>

namespace rtcus_navigation
{
namespace goal_reach_detection
{
using namespace rtcus_navigation;

template<typename StateType, typename GoalType>
  class ReachedGoalPredicateBase : public ReachedGoalPredicate<StateType, GoalType>
  {
  protected:
    bool reach_latch_;
    bool goal_reached_;
    double acceptance_radious_;

  public:
    ReachedGoalPredicateBase() :
        reach_latch_(false), goal_reached_(false), acceptance_radious_(0.0)
    {

    }
    virtual void init()
    {
      reset();
    }
    virtual void reset()
    {
      reach_latch_ = false;
      goal_reached_ = false;
      acceptance_radious_ = 0.0;

      ros::NodeHandle& nh = this->component_node_;

      if (!nh.getParam("acceptance_radious", acceptance_radious_))
        nh.setParam("acceptance_radious", acceptance_radious_);
      if (!nh.getParam("reach_latch", reach_latch_))
        nh.setParam("reach_latch", reach_latch_);

    }
    virtual float distance(const StateType& current_state, const GoalType& goal)=0;

    virtual bool isGoalReached(const StateType& current_state, const GoalType& goal)
    {
      if (distance(current_state, goal) < acceptance_radious_)
      {
        goal_reached_ = true;
      }
      else if (!reach_latch_)
        goal_reached_ = false;

      return goal_reached_;
    }

    virtual ~ReachedGoalPredicateBase()
    {
    }
  };
}
}

#endif /* DEFAULT_GOAL_REACH_DETECTION_H_ */
