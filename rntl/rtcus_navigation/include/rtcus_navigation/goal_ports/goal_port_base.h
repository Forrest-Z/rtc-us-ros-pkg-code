/*
 * goal_port_base.h
 *
 *  Created on: Jun 20, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef GOAL_PORT_BASE_H_
#define GOAL_PORT_BASE_H_

#include <rtcus_navigation/common.h>
#include <rtcus_navigation/goal_port.h>
#include <rtcus_assert/rtcus_assert.h>

namespace rtcus_navigation
{
namespace goal_ports
{

template<typename GoalType>
  class DefaultMsgsGoalPortBase : public GoalPort<GoalType>
  {

  protected:
    virtual void topic_subscribe()=0;
    ros::Subscriber goal_sub_;
    StampedData<GoalType> goal_;
    std::string goal_frame_;
    std::string robot_base_local_frame_;
    bool latched_;

  public:
    DefaultMsgsGoalPortBase() :
        GoalPort<GoalType>::GoalPort(), latched_(false)
    {

    }

    virtual void init(const std::string& reference_frame, const std::string& robot_base_frame)
    {
      reset();
      robot_base_local_frame_ = robot_base_frame;
      goal_frame_ = reference_frame;
      this->topic_subscribe();
    }

    virtual void reset()
    {
      latched_ = false;
      if (!this->component_node_.getParam("latched_goal_estimation", latched_))
        this->component_node_.setParam("latched_goal_estimation", latched_);
    }

    virtual std::string getReferenceFrame() const
    {
      return goal_frame_;
    }

    virtual ~DefaultMsgsGoalPortBase()
    {

    }
    ;

    virtual void pushGoal(const Stamped<GoalType>& goal)
    {

      RTCUS_ASSERT_MSG(
          ARE_SAME_FRAMES(goal.getFrameId(), this->goal_frame_)
              || ARE_SAME_FRAMES(goal.getFrameId(), this->robot_base_local_frame_),
          "For now, the goal only can be expressed in the robot local frame [%s] or the global frame [%s]",
          this->robot_base_local_frame_.c_str(), this->goal_frame_.c_str());

      ros::Time received_time = ros::Time::now();
      this->goal_ = StampedData<GoalType>(goal.getConstData(), goal.getStamp(), goal.getFrameId());
      this->onGoalNotification(*this, received_time);
    }

    virtual void getLastGoalNotification(StampedData<GoalType>& goal) const
    {
      goal = this->goal_;
    }

    //this consider non moving obstacles since returns the same than the last goal notification
    virtual void getGoalEstimation(ros::Time time, StampedData<GoalType>& goal) const
    {
      if (latched_)
      {
        StampedData<GoalType> goal_estimation;
        goal_estimation = goal_;
        goal_estimation.setStamp(time);
        goal = goal_estimation;
      }
      else
        goal = goal_;
    }
  };
}
}

#endif /* GOAL_PORT_BASE_H_ */
