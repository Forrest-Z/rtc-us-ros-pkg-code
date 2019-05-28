/*
 * non_time_correction_state_estimation.h
 *
 *  Created on: Sep 12, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef NON_TIME_CORRECTION_STATE_ESTIMATION_H_
#define NON_TIME_CORRECTION_STATE_ESTIMATION_H_

#include <rtcus_navigation/state_estimation.h>

namespace rtcus_navigation
{
namespace state_estimation
{
using namespace boost;
using namespace rtcus_stamp;
using namespace rtcus_motion_models;
using namespace rtcus_navigation;

/** \brief This class takes the last measurements of state and uses it like the current state of the system.
 * This is the typical behavior that most of systems uses. (Without motion prediction used in the PTC method)
 * */
template<typename StateType, typename ActionType, typename TimeModel = ROSTimeModel>
  class NonTimeCorrectionStateEstimation : public StateEstimation<StateType, ActionType, TimeModel>
  {
    USING_TIME_MODEL(TimeModel);

  private:
    StampedData<StateType, TTime> last_state_reading_;
    StampedData<ActionType, TTime> last_command_;
    std::string reference_frame_;
    ActionType stopAction_;
  public:

    NonTimeCorrectionStateEstimation() :
        StateEstimation<StateType, ActionType, TimeModel>::StateEstimation()
    {

    }

    virtual ~NonTimeCorrectionStateEstimation()
    {
    }

    virtual void init(const AbstractNavigationNode& navigation_node, const ActionType& stopAction)
    {
      ROS_INFO("Initializating the State Estimation Component (No Time Correction)");
      reference_frame_ = navigation_node.getConfig().reference_frame;
      stopAction_ = stopAction;
      reset();
    }

    virtual void reset()
    {
      last_command_.setFrameId(reference_frame_);
      last_command_.setStamp(TTime::now());
      last_command_.copyFromData(stopAction_);
    }

    virtual StampedData<StateType, TTime> computeStateEstimation(const TTime& application_time,
                                                                 const Stamped<StateType, TTime>& last_state_reading)
    {
      this->last_state_reading_.copyFromData(last_state_reading.getConstData());
      this->last_state_reading_.setStamp(application_time);
      this->last_state_reading_.setFrameId(reference_frame_);
      return last_state_reading_;
    }

    virtual bool getLastStateEstimation(StampedData<StateType, TTime>& estimation) const
    {
      estimation = last_state_reading_;
      return true;
    }

    virtual bool getPastState(const TTime& stamp, StateType& past_state) const
    {
      past_state = last_state_reading_.getConstData();
      return true;
    }

    virtual void registerAction(const ActionType& cmd_vel, const TTime& application_time)
    {
      this->last_command_ = cmd_vel;

    }
    virtual bool getLastCommand(Stamped<ActionType, TTime>& last_command) const
    {
      last_command = StampedData<ActionType, TTime>(this->last_command_.getConstData(), last_command_.getStamp(),
                                                    last_command_.getFrameId());
      return true;
    }
  };

}
}

#endif /* NON_TIME_CORRECTION_STATE_ESTIMATION_H_ */
