/*
 * default_goal_port.h
 *
 *  Created on: Jun 20, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef DEFAULT_GOAL_PORT_H_
#define DEFAULT_GOAL_PORT_H_

#include <rtcus_navigation/goal_ports/goal_port_base.h>

namespace rtcus_navigation
{
namespace goal_ports
{

template<typename RosMsgGoal>
  class DefaultGoalPort : public DefaultMsgsGoalPortBase<RosMsgGoal>
  {

  protected:
    virtual void topic_subscribe()
    {
      ros::NodeHandle& nh = this->node_;
      this->goal_sub_ = nh.subscribe<RosMsgGoal>("goal", 1,
                                                 boost::bind(&DefaultGoalPort<RosMsgGoal>::goalMsgCallback, this, _1));
    }

    void goalMsgCallback(const boost::shared_ptr<const RosMsgGoal>& goal_pose_msg)
    {
      std::string goal_frame;
      ros::Time goal_time;

      StampedData<RosMsgGoal> goal(*goal_pose_msg, goal_time, goal_frame);
      this->pushGoal(goal);
    }

  public:

    DefaultGoalPort() :
        DefaultMsgsGoalPortBase<RosMsgGoal>::DefaultMsgsGoalPortBase()
    {

    }
    virtual ~DefaultGoalPort()
    {
    }

  };
}

}

#endif /* DEFAULT_GOAL_PORT_H_ */
