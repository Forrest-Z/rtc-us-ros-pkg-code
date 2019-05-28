/*
 * core.h
 *
 *  Created on: Jun 18, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef RTCUS_NAVIGATION_CORE_H_
#define RTCUS_NAVIGATION_CORE_H_

#include <rtcus_navigation/navigation_component.h>

namespace rtcus_navigation
{
class ROSTimeModel;

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShape,
    typename KynoDynamicDescription, typename TimeModel = ROSTimeModel>
  class NavigationNode;

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShape,
    typename KynoDynamicDescription, typename TimeModel = ROSTimeModel>
  class NavigationPlanner;

template<typename GoalType, typename TimeModel = ROSTimeModel>
  class GoalPort;

template<typename StateType, typename TimeModel = ROSTimeModel>
  class StatePort;

template<typename StateType, typename ActionType, typename TimeModel = ROSTimeModel>
  class StateEstimation;

template<typename ObstaclesType, typename TimeModel = ROSTimeModel>
  class WorldPerceptionPort;

template<typename ActionType, typename TimeModel = ROSTimeModel>
  class ActionPort;

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShape,
    typename KynoDynamicDescription, typename TimeModel = ROSTimeModel>
  class TaskStatus;

class NavigationNodeComponent;

class ROSTimeModel
{
};

template<typename TimeModel>
  class TimeModelTraits
  {
  public:
    typedef ros::Time TTime;
    typedef ros::Duration TDuration;
  };

template<>
  class TimeModelTraits<ROSTimeModel>
  {
  public:
    typedef ros::Time TTime;
    typedef ros::Duration TDuration;
  };

//------------------------------------------------------------------
template<typename TDestine>
  class CovariantTraits
  {
    template<typename TSource>
        static boost::shared_ptr<TDestine> CovariantDecorator(boost::shared_ptr<TSource> src);
  };

template<typename TSrc, typename TDest>
  class ExplicitCovarianceTraits
  {
  public:
    template<typename InputType>
      static boost::shared_ptr<TDest> CovariantDecorator(boost::shared_ptr<InputType> input)
      {
        return rtcus_navigation::CovariantTraits<TDest>::CovariantDecorator(boost::static_pointer_cast<TSrc>(input));
      }
  };

//------------------------------------------------------------------

#define USING_TIME_MODEL(TimeModel) \
  typedef typename rtcus_navigation::TimeModelTraits<TimeModel>::TDuration TDuration; \
  typedef typename rtcus_navigation::TimeModelTraits<TimeModel>::TTime TTime

#define USING_NAVIGATION_TYPES(NavigationNodeOrPlannerType) \
    typedef typename NavigationNodeOrPlannerType::TStateType TStateType; \
    typedef typename NavigationNodeOrPlannerType::TObstaclesType TObstaclesType;\
    typedef typename NavigationNodeOrPlannerType::TActionType TActionType; \
    typedef typename NavigationNodeOrPlannerType::TGoalType TGoalType; \
    typedef typename NavigationNodeOrPlannerType::TRobotShapeType TRobotShapeType; \
    typedef typename NavigationNodeOrPlannerType::TKinoDynamicDescription TKinoDynamicDescription; \
    typedef typename NavigationNodeOrPlannerType::TTaskStatus TTaskStatus; \
    typedef typename NavigationNodeOrPlannerType::TTimeModel TTimeModel; \
    typedef typename rtcus_navigation::TimeModelTraits<TTimeModel>::TDuration TDuration; \
    typedef typename rtcus_navigation::TimeModelTraits<TTimeModel>::TTime TTime;

#define DECLARE_NAVIGATION_PLANNER_TYPES() \
    typedef rtcus_navigation::NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShape, \
           KinoDynamicDescription, TimeModel> TNavigationNode; \
typedef boost::shared_ptr<TNavigationNode> TNavigationNodePtr; \
typedef rtcus_navigation::NavigationPlanner<StateType, ObstaclesType, ActionType, GoalType, RobotShape, \
    KinoDynamicDescription, TimeModel> TNavigationPlanner; \
    typedef rtcus_navigation::TaskStatus <StateType, ObstaclesType, ActionType, GoalType, RobotShape, \
        KinoDynamicDescription, TimeModel> TTaskStatus; \
USING_TIME_MODEL (TimeModel)

#define DECLARE_NAVIGATION_PLANNER_TYPES_EXPLICIT(StateType, ObstaclesType, ActionType, GoalType, RobotShape, KinoDynamicDescription, TimeModel) \
typedef rtcus_navigation::NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShape, \
        KinoDynamicDescription, TimeModel> TNavigationNode; \
typedef boost::shared_ptr<TNavigationNode> TNavigationNodePtr; \
typedef rtcus_navigation::NavigationPlanner<StateType, ObstaclesType, ActionType, GoalType, RobotShape, \
 KinoDynamicDescription, TimeModel> TNavigationPlanner;\
 typedef rtcus_navigation::TaskStatus <StateType, ObstaclesType, ActionType, GoalType, RobotShape, \
         KinoDynamicDescription, TimeModel> TTaskStatus;  \
 USING_TIME_MODEL (TimeModel)
}

#endif /* RTCUS_NAVIGATION_CORE_H_ */
