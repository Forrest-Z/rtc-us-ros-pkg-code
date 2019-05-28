/*
 *
 *  Created on: Jun 13, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef ADAPTABLE_STATE_PORT_H_
#define ADAPTABLE_STATE_PORT_H_

#include <rtcus_navigation/state_ports/default_state_port.h>
#include <rtcus_nav_msgs/DynamicState2D.h>
#include <nav_msgs/Odometry.h>
#include <rtcus_stamp/stamped.h>
#include <tf/tf.h>
#include <rtcus_navigation/common.h>

namespace rtcus_navigation
{
namespace state_ports
{

using namespace rtcus_motion_models;
using namespace nav_msgs;
using namespace rtcus_nav_msgs;

template<typename StateType, typename InputRosMsg>
  class AdaptableStateCorrectionPortBase : public DefaultStatePort<StateType>
  {
  private:
    void state_correctionCB(const boost::shared_ptr<const InputRosMsg>& state_correction_msg)
    {
      std::string frame_id;
      ros::Time stamp;
      StateType state;

      fromInputRosMsgToInternalState(*state_correction_msg, state, stamp, frame_id);

      if (ARE_DISTINCT_FRAMES(frame_id, this->global_frame))
      {
        ROS_WARN(
            "State Port Module. invalid state reading and it will be ignored. \n This is because the provided data header frame [%s] does not match with frame global state frame[%s]", frame_id.c_str(), this->global_frame.c_str());
        return;
      }

      this->pushState(StampedData<StateType>(state, stamp, frame_id));
    }

  protected:

    /**
     * \brief converts from InputRosMsg to StateType. It also provides data about the input msg frame_id and stamp
     * */
    virtual void fromInputRosMsgToInternalState(const InputRosMsg& msg, StateType& output_state,
                                                ros::Time& output_stamp, std::string& frame_id)=0;

  public:

    AdaptableStateCorrectionPortBase() :
        DefaultStatePort<StateType>()
    {

    }

    virtual void init(std::string reference_frame)
    {
      DefaultStatePort<StateType>::init(reference_frame);

      ros::NodeHandle& nh = this->node_;
      this->state_correction_sub_ = nh.subscribe<InputRosMsg>(
          "state_correction", 1,
          boost::bind(&AdaptableStateCorrectionPortBase<StateType, InputRosMsg>::state_correctionCB, this, _1));
    }

    virtual ~AdaptableStateCorrectionPortBase()
    {
    }
  };
}
}

#endif
