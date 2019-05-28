/*
 * task_status.h
 *
 *  Created on: Dec 8, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef TASK_STATUS_H_
#define TASK_STATUS_H_

#include <rtcus_navigation/core.h>

namespace rtcus_navigation
{
using namespace rtcus_stamp;

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShape,
    typename KynoDynamicDescription, typename TimeModel>
  class TaskStatus
  {
    USING_TIME_MODEL(TimeModel);

    typedef TaskStatus<StateType, ObstaclesType, ActionType, GoalType, RobotShape, KynoDynamicDescription, TimeModel> TTaskStatus;
  public:
    TaskStatus() :
        current_stage(0), valid_state_estimation(false), valid_goal_estimation(false), goal_reached_prediction(false), obstacles_computed(
            false), collision_prediction(false)
    {
    }

  private:

    void init(TTime application_time)
    {
      this->application_time_ = application_time;
    }

    int current_stage;
    TTaskStatus& next()
    {
      current_stage++;
      return *this;
    }

    //TODO: make inline getters and setters for all those methods and remove the friendship between classes
    friend class NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShape, KynoDynamicDescription,
        TimeModel> ;

    TTime application_time_;
    TTime planning_time_;
    bool valid_state_estimation;
    bool valid_goal_estimation;
    bool goal_reached_prediction;
    bool obstacles_computed;
    bool collision_prediction;

    /**  \brief x(t_x) */
    Stamped<StateType, TTime> last_state_reading_;

    /** \brief g(t_g) */
    StampedData<GoalType, TTime> last_goal_reading_;

    /**  \brief o(t_o)  */
    Stamped<ObstaclesType, TTime> last_obstacle_reading_;

    /*-------------------- application time (t_u) info -------------------*/
    /** \brief x(t_o) */
    StampedData<StateType, TTime> state_at_obstacle_reading_;

    /** \brief x(t_u) */
    StampedData<StateType, TTime> state_estimation_;

    /** \brief o(t_u) */
    Stamped<ObstaclesType, TTime> resulting_obstacles_;

    /** \brief g(t_u) */
    StampedData<GoalType, TTime> resulting_goal_;

    /**\brief s(t_u)*/
    StampedData<RobotShape, TTime> resulting_shape_;

  public:
    inline const TTime& getPlanningTime() const
    {
      return this->planning_time_;
    }

    inline const TTime& getExpectedApplicationTime() const
    {
      return this->application_time_;
    }

    inline const Stamped<StateType, TTime>& getLastStateReading() const
    {
      return last_state_reading_;
    }

    inline const StampedData<StateType, TTime> getStateAtLastObstacleReading() const
    {
      return state_at_obstacle_reading_;
    }

    inline const StampedData<StateType, TTime>& getStateEstimation() const
    {
      return state_estimation_;
    }
    inline const Stamped<ObstaclesType, TTime>& getComputedObstacles() const
    {
      return resulting_obstacles_;
    }
    inline const StampedData<GoalType, TTime>& getComputedGoal() const
    {
      return resulting_goal_;
    }
  };
}

#endif /* TASK_STATUS_H_ */
