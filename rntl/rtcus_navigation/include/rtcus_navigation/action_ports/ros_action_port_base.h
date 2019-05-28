/*
 * default_ros_msg_action_port.h
 *
 *  Created on: Apr 17, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef ROS_MSG_ACTION_PORT_BASE_H_
#define ROS_MSG_ACTION_PORT_BASE_H_

#include <rtcus_navigation/action_port.h>
#include <rtcus_navigation/common.h>
#include <rtcus_navigation/ActionPortConfig.h>
#include <dynamic_reconfigure/server.h>

namespace rtcus_navigation
{
namespace action_ports
{

using namespace rtcus_stamp;
using namespace std;
using namespace rtcus_motion_models;

template<typename ActionType, typename RosMsgType, typename TimeModel = ROSTimeModel>
  class ROSActionPortBase : public ActionPort<ActionType, TimeModel>
  {
    USING_TIME_MODEL(TimeModel);
  protected:
    ros::Publisher vel_pub_;

    ActionPortConfig config_;
    dynamic_reconfigure::Server<ActionPortConfig> configure_server_;

    virtual void convertActionTypeToMsgType(const ActionType& input, RosMsgType& output,
                                            TTime expected_application_time)=0;

  protected:
    ActionPortConfig configure_callback(const ActionPortConfig& config, int level)
    {
      this->config_ = config;
      return this->config_;
    }
  public:
    ROSActionPortBase(std::string output_topic_name = "cmd_vel") :
        ActionPort<ActionType, TimeModel>::ActionPort(), configure_server_(this->getComponentNode())
    {
      ros::NodeHandle & nh = this->node_;
      this->configure_server_.setCallback(
          boost::bind(&ROSActionPortBase<ActionType, RosMsgType, TimeModel>::configure_callback, this, _1, _2));
      this->config_.output_topic_name = output_topic_name;
      vel_pub_ = nh.advertise<RosMsgType>(this->config_.output_topic_name, 1);
    }

    virtual void init()
    {
      reset();
    }

    virtual void reset()
    {
      if (!this->component_node_.getParam("application_time_oracle", this->config_.application_time_oracle))
        this->component_node_.setParam("application_time_oracle", this->config_.application_time_oracle);
      else
        ROS_INFO("Action Port. Setting the expected time delay to %lf secs", this->config_.application_time_oracle);
    }

    virtual ~ROSActionPortBase()
    {
    }

    virtual bool sendAction(const ActionType& cmd_vel, TTime expected_application_time)
    {
      RosMsgType out_cmd_vel;
      convertActionTypeToMsgType(cmd_vel, out_cmd_vel, expected_application_time);
      vel_pub_.publish(out_cmd_vel);
      return true;
    }

    virtual void getStopAction(ActionType& stopAction) const
    {
      stopAction = ActionType();
    }

    virtual ros::Duration getExpectedTimeDelay() const
    {
      return ros::Duration(this->config_.application_time_oracle);
    }

  };

}
}
#endif /* DEFAULT_ROS_MSG_ACTION_PORT_H_ */
