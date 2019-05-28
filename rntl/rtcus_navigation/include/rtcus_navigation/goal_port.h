/*
 * goal_port.h
 *
 *  Created on: Apr 6, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef GOAL_PORT_H_
#define GOAL_PORT_H_

#include <rtcus_navigation/core.h>

namespace rtcus_navigation
{

using namespace rtcus_stamp;

/*! \brief the responsability of this class is to store the goal information in a fixed reference frame.
 * */
template<typename GoalType, typename TimeModel>
  class GoalPort : public NavigationNodeComponent
  {
  private:
    USING_TIME_MODEL (TimeModel);

  public:

    virtual ~GoalPort()
    {
    }

    virtual void reset()=0;
    virtual void init(const std::string& reference_frame, const std::string& robot_base_frame)=0;

    /*! \brief get the estimation of the current goal at the instant time.
     *  \param time specifies the time in which the goal has to be estimated. This is interesting when the obstacle is moving.
     *  \returns the estimated goal. Stamped in a reference frame in an specific instant ( it is typically the last notification stamp)
     *
     *  \remarks state. It is important to understand that the goal
     *  was declared in a reference frame in the past. The reference frame can be usually the mobile robot base in the past or
     *  other fixed frame.
     */
    virtual void getGoalEstimation(TTime time, StampedData<GoalType, TTime>&) const=0;

    /*!
     *  \brief get the last goal notification stamped with the reference frame and timestamp.
     * */
    virtual void getLastGoalNotification(StampedData<GoalType, TTime>&) const=0;

    /*! \brief introduces dynamically a goal msg on the port.
     *  \remarks this method is not typically used since the goal ports typically get data from ROS topics.
     * */
    virtual void pushGoal(const rtcus_stamp::Stamped<GoalType, TTime>& goal)=0;

    /*! \brief this event is invoked when a new goal is received.
     *  \remarks it should be implemented by the specific goal port class implementation.
     * */
    boost::signal<void(GoalPort<GoalType, TimeModel>& sender, TTime receivedTime)> onGoalNotification;

  protected:
    GoalPort() :
        NavigationNodeComponent(tGoalPort)
    {
    }
  };

}

#endif /* GOAL_PORT_H_ */
