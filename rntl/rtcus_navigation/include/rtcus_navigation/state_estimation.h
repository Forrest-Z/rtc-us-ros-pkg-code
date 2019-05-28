/*
 * state_estimation.h
 *
 *  Created on: Jul 2, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef STATE_ESTIMATION_H_
#define STATE_ESTIMATION_H_

#include <rtcus_navigation/core.h>
#include <rtcus_navigation/common.h>
#include <rtcus_motion_models/motion_models.h>
#include <rtcus_navigation/abstract_navigation_node.h>

namespace rtcus_navigation
{
using namespace boost;
using namespace rtcus_stamp;
using namespace rtcus_motion_models;

/**
 * \brief This core class interface defines a component that should model how the robot will move in the future.
 * This is used to get the application time state  ^x(t_u)
 *
 * TODO: This class interface is expected to be merged with the state port interface.
 * */
template<typename StateType, typename ActionType, typename TimeModel>
  class StateEstimation : public NavigationNodeComponent
  {
    USING_TIME_MODEL(TimeModel);

  public:

    StateEstimation() :
        NavigationNodeComponent(tStateEstimation)
    {

    }

    virtual void reset()=0;
    virtual void init(const AbstractNavigationNode& navigation_node, const ActionType& initial_action)=0;
    //virtual void setMotionModel(
    //    const boost::shared_ptr<MotionModel<StateType, ActionType, TDuration, TTime> >& motion_model)=0;

    virtual ~StateEstimation()
    {
    }

    //TODO: document this method
    virtual StampedData<StateType, TTime> computeStateEstimation(const TTime& application_time,
                                                                 const Stamped<StateType, TTime>& last_state_reading)=0;
    virtual bool getLastStateEstimation(StampedData<StateType, TTime>&) const=0;

    /*! \brief return a recorded past state.
     * \param stamp the stamp of the desired state in the past
     * \param past_state the output of the function.
     * \returns false if the past state correction has not been found
     *
     * \TODO: recomendations for implementations could take into account the robot motion model and the control history. Additionally others implementations
     * could also use interpolations of states to compute an state not recorded at such timestamp explicitly
     * */
    virtual bool getPastState(const TTime& stamp, StateType& past_state) const =0;

    virtual void registerAction(const ActionType& cmd_vel, const TTime& application_time)=0;
    virtual bool getLastCommand(Stamped<ActionType, TTime>& last_command) const=0;

  };
}
#endif /* STATE_ESTIMATION_H_ */
