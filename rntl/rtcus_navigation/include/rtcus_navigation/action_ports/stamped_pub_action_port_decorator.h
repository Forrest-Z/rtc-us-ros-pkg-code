/*
 * stamped_pub_action_port_decorator.h
 *
 *  Created on: Jan 13, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef STAMPED_PUB_ACTION_PORT_DECORATOR_H_
#define STAMPED_PUB_ACTION_PORT_DECORATOR_H_

#include <rtcus_navigation/action_port.h>
using namespace rtcus_nav_msgs;

namespace rtcus_navigation
{
namespace action_ports
{
//TODO: Yet to implement the topic publishing
template<typename ActionType, typename TimeModel = ROSTimeModel>
  class StampedPubActionPortDecorator : public ActionPort<ActionType, TimeModel>
  {
    USING_TIME_MODEL(TimeModel);
  protected:
    boost::shared_ptr<ActionPort<ActionType, TimeModel> > decorated_;

    void forward_action_sent_event(ActionPort<ActionType, TimeModel>& sender, const TTime& sendingTime)
    {
      this->onActionSent(this, sendingTime);
    }

  public:
    StampedPubActionPortDecorator(shared_ptr<ActionPort<ActionType, TimeModel> > decorated)
    {
      this->decorated_ = decorated;
      this->decorated_->onActionSent.connect(
          boost::bind(StampedPubActionPortDecorator<ActionType, TimeModel>::forward_action_sent_event, this, _1, _2));
    }

    virtual bool sendAction(const ActionType& cmd_vel, TTime expected_application_time)
    {
      return decorated_->sendAction(cmd_vel, expected_application_time);
    }
    virtual void reset()
    {
      decorated_->reset();
    }
    virtual void init()
    {
      decorated_->init();
    }

    virtual void getStopAction(ActionType& stopAction) const
    {
      decorated_->getStopAction(stopAction);
    }

    virtual TDuration getExpectedTimeDelay() const
    {
      return decorated_->getExpectedTimeDelay();
    }

    virtual ~ActionPort()
    {
    }

  };
}
}

#endif /* STAMPED_PUB_ACTION_PORT_DECORATOR_H_ */
