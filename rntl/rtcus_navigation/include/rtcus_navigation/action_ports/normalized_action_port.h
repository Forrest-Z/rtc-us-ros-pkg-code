/*
 * normalized_action_port.h
 *
 *  Created on: Dec 17, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef NORMALIZED_ACTION_PORT_H_
#define NORMALIZED_ACTION_PORT_H_
#include <rtcus_navigation/action_port.h>

namespace rtcus_navigation
{
namespace action_ports
{

/**
 * \brief This action port decorates an external action port and normalize the output given the
 * kinodynamic capabilities of the system.
 * */
template<typename ActionType>
  class NormalizedActionPort : ActionPort<ActionType>
  {

    boost::shared_ptr<ActionPort<ActionType> > decorated_;
  public:
    NormalizedActionPort(boost::shared_ptr<ActionPort<ActionType> > decorated)
    {
      decorated_ = decorated;
    }

  };
}
;
}

#endif /* NORMALIZED_ACTION_PORT_H_ */
