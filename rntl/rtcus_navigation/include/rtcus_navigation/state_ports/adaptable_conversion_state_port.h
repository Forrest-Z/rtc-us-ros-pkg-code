/*
 * adaptable_conversion_state_port.h
 *
 *  Created on: Aug 7, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef ADAPTABLE_CONVERSION_STATE_PORT_H_
#define ADAPTABLE_CONVERSION_STATE_PORT_H_

#include <rtcus_navigation/state_ports/adaptable_state_correction_port.h>

namespace rtcus_navigation
{
namespace state_ports
{
template<typename StateType, typename InputRosMsg>
  class AdaptableConversionStatePort : public AdaptableStateCorrectionPortBase<StateType, InputRosMsg>
  {
  protected:
    virtual void fromInputRosMsgToInternalState(const InputRosMsg& msg, StateType& output_state,
                                                ros::Time& output_stamp, std::string& frame_id)
    {
      rtcus_conversions::Conversions::convert(msg, output_state);

      if (isHeadedRosMsg(msg, output_stamp, frame_id))
        setHeaderRosMsg(output_state, output_stamp, frame_id);
      else
      {
#define msg_error "State Correction Port. This state port implementation does not support input msgs without header"
        ROS_ERROR(msg_error);
        throw ros::Exception(msg_error);
      }
    }
  public:
    virtual ~AdaptableConversionStatePort()
    {
    }
  };
}
}

#endif /* ADAPTABLE_CONVERSION_STATE_PORT_H_ */
