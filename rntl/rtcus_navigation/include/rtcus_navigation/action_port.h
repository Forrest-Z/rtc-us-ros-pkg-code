/*
 * action_port.h
 *
 *  Created on: Apr 6, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef ACTION_PORT_H_
#define ACTION_PORT_H_
#include <rtcus_navigation/core.h>

namespace rtcus_navigation
{
/*\brief Contains a set of strategies about how to send actions to the mobile platfform: actuator or a closer-lower-level software to the platfform.*/
namespace action_ports
{
}

/*\brief Defines how the actions has to be sent to the mobile platform: actuator or a closer-lower-level software to the platfform. It should
 * describe all the interesting knowledge about the */
template<typename ActionType, typename TimeModel>
  class ActionPort : public NavigationNodeComponent
  {
    USING_TIME_MODEL(TimeModel);

  public:
    virtual bool sendAction(const ActionType& cmd_vel, TTime expected_application_time)=0;
    virtual void reset()=0;
    virtual void init()=0;

    //TODO: This should take into account the robot kinodynamic. Move this? Maybe not.
    //It does not matter if the robot does not take the stop action instantaneusly. This is the stop action.
    //we will see in the future if change this
    virtual void getStopAction(ActionType&) const=0;

    //TODO: This should go to the TimeModel to aggregate both: planning time delays and actuator delays
    virtual TDuration getExpectedTimeDelay() const=0;

    virtual ~ActionPort()
    {
    }

    boost::signal<void(const ActionPort<ActionType, TimeModel>& sender, const TTime& sendingTime)> onActionSent;

  protected:
    ActionPort() :
        NavigationNodeComponent(tActionPort)
    {
    }
  };

}

#endif /* ACTION_PORT_H_ */
