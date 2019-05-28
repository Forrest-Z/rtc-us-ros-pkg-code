/*
 * adaptable_goal_port.h
 *
 *  Created on: Jun 19, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef ADAPTABLE_GOAL_PORT_H_
#define ADAPTABLE_GOAL_PORT_H_

#include <rtcus_navigation/goal_ports/goal_port_base.h>
#include <rtcus_assert/rtcus_assert.h>

namespace rtcus_navigation
{
namespace goal_ports
{
template<typename GoalType, typename RosMsgGoal>
  class AdaptableGoalPort : public DefaultMsgsGoalPortBase<GoalType>
  {
  private:
    void goalCB(const boost::shared_ptr<const RosMsgGoal>& goal_pose_msg)
    {
      StampedData<GoalType> goal;
      process_message(goal, goal_pose_msg);
      this->pushGoal(goal);

    }
  protected:
    virtual void topic_subscribe()
    {
      ros::NodeHandle& nh = this->node_;
      this->goal_sub_ = nh.subscribe<RosMsgGoal>(
          "goal", 1, boost::bind(&AdaptableGoalPort<GoalType, RosMsgGoal>::goalCB, this, _1));
      ROS_WARN("Adaptable GoalPort. Subscribing to the goal port 'goal'");
    }

    virtual void convert_msg(const RosMsgGoal& goal_pose_msg, GoalType& goal)=0;

    virtual void process_message(StampedData<GoalType>& goal, const boost::shared_ptr<const RosMsgGoal>& goal_pose_msg)
    {
      std::string goal_frame;
      ros::Time goal_time;
      if (isHeadedRosMsg(*goal_pose_msg, goal_time, goal_frame))
      {

        RTCUS_ASSERT_MSG(
            ARE_SAME_FRAMES(goal_frame, this->goal_frame_) || ARE_SAME_FRAMES(goal_frame, this->robot_base_local_frame_),
            "For now, the goal only can be expressed in the robot local frame [%s] or the global frame [%s]", this->robot_base_local_frame_.c_str(), this->goal_frame_.c_str());

        this->convert_msg(*goal_pose_msg, goal.getData());
        goal.setStamp(goal_time);
        goal.setFrameId(this->goal_frame_);
      }
      else
      {
        throw ros::Exception("No stamped goal is allowed in this implementation");
      }
    }

  public:

    AdaptableGoalPort() :
        DefaultMsgsGoalPortBase<GoalType>::DefaultMsgsGoalPortBase()
    {

    }
    virtual ~AdaptableGoalPort()
    {
    }

    virtual void reset()
    {
      DefaultMsgsGoalPortBase<GoalType>::reset();
    }

  };
}

}

#endif /* ADAPTABLE_GOAL_PORT_H_ */
