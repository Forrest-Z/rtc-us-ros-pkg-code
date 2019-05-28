/*
 *
 *  Created on: 18/12/2011
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef LOCAL_PLANNER_BASE_H_
#define LOCAL_PLANNER_BASE_H_
#include <rtcus_navigation/core.h>

namespace rtcus_navigation
{

/* \brief inspired in nav_core::BaseLocalPlanner. This is a ROS-specific interface so if you have a ros-independent algorithm it could be wrapped by a subclass of this NavigationPlannerBase following the adapter software pattern.
 * always working in the local world to make easier the implementation.
 * \remarks typical information to compute the next command (ie: current speed, goal position, state estimation, etc.) should be handled using the host navigation node reference manually.
 * No time correction has to be implemented inside the planner.External predictors are supposed to give all the information for the current instant. The temporal correction if exist is handled in the navigation node class.
 * */
template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShape,
    typename KinoDynamicDescription, typename TimeModel>
  class NavigationPlanner : public NavigationNodeComponent
  {
  public:
    DECLARE_NAVIGATION_PLANNER_TYPES();

    NavigationPlanner() :
        NavigationNodeComponent(tNavigationPlanner)
    {
    }

    virtual ~NavigationPlanner()
    {
    }

    virtual void reset()=0;
    virtual void init(TNavigationNode& host_node)=0;

    /**
     * \brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
     * \param cmd_vel Will be filled with the velocity command to be passed to the robot base
     * \return True if a valid velocity command was found, false otherwise
     */
    virtual bool computeVelocityCommands(const ObstaclesType& obstacles, const GoalType& goal,
                                         const StateType& local_state, ActionType& resulting_action)=0;

    // ============ EVENTS =========

    boost::signal<void()> OnParametersUpdate;
    /**
     *  \brief This event should by called by the implementation each time its config had been updated.
     * */
    boost::signal<void(TNavigationPlanner& sender)> onConfigUpdated;
    boost::signal<void(TNavigationPlanner& sender)> onPrecomputeCommand;
    boost::signal<
        void(TNavigationPlanner& sender, const ActionType& best_cmd, const StateType& local_state,
             const GoalType& local_goal)> onPostComputeCommand;

  };
}

#endif /* LOCALPLANNERBASE_H_ */
