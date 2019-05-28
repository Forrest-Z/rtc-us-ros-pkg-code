/*
 * 
 *  Created on: 18/12/2011
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville (Spain) - 2012
 *      License: GPLv3
 */

#ifndef MOTION_MODELS_H_
#define MOTION_MODELS_H_

#include<rtcus_stamp/stamped.h>
#include <rtcus_compositions/state_composer.h>
#include <vector>

namespace rtcus_motion_models
{

using namespace std;
using namespace rtcus_stamp;
using namespace rtcus_compositions;

/*!
 * @brief This class defines a motion model of a robot that allow predicting future states given a set of actions.
 *
 * The time and duration template parameter should implement the operators
 * Duration = TTime - TTime
 * int = TTime / Duration
 * TTime = TTime
 * TTime = TTime + Duration
 * zeroDuration Constructor Duration(double secs)
 * Duration = Duration * float
 * Duration.toSec() -> double
 *
 * TTime > TTime
 * Ttime < TTime
 * */
template<typename StateType, typename ActionType, typename Duration = ros::Duration, typename TTime = ros::Time>
  class MotionModel
  {

  protected:
    virtual void transfer_function(const StateType& initial_state, const ActionType& action,
                                   const Duration action_duration, StateType& final_state) const=0;
  public:

    /*!
     * @fn virtual void MotionModel::predict(const StateType& initial_local_state, const ActionType& action, const Duration simulation_time,
     StateType& final_state)

     * This is the Transfer state function that is defined by the specific motion model.
     * It predicts the future state of the robot given an input Action. Abstract.
     * @param initial_state the initial state of the robot with a time stamp
     * @param action the action to apply to the robot from the initial time stamp
     * @param action_duration the duration of the action application
     * @param final_state the output param, it is the output of the function. It contains the predictd state given the inputs of the function.
     * */
    //1- FINALSTATE SINGLE-ACTION NONSTAMPED N/A(SYNC)
    void predictState(const StateType& initial_state, const ActionType& action, const Duration action_duration,
                      StateType& final_state) const;

    /*!
     * @fn virtual void MotionModel::predict(const StateType& initial_local_state, const ActionType& action, const Duration simulation_time,
     StateType& final_state)

     * This method uses the transfer function defined by the specific motion models and return the stamped final state.
     * It predicts the future state of the robot given an input Action. Abstract.
     * @param initial_state the initial state of the robot with a time stamp
     * @param action the action to apply to the robot from the initial time stamp
     * @param action_duration the duration of the action application
     * @param final_state the output param, it is the output of the function. It contains the predictd state given the inputs of the function.
     * */
    //2- FINALSTATE SINGLE-ACTION STAMPED N/A(SYNC)
    void predictState(const Stamped<StateType>& initial_state, const ActionType& action, const Duration action_duration,
                      Stamped<StateType>& final_state) const;

    /*!@brief Just apply the transfer function multiple times according the given actions in a regular time period.
     * @param action_duration the duration of one action
     * */
    //3- FINALSTATE MLTIPLEACTIONS NONSTAMPED SYNC
    void predictState(const StateType& initial_state, const vector<ActionType>& action_history,
                      Duration action_duration, StateType& final_state) const;

    /*!
     * @brief Just apply the transfer function multiple times according the given actions in a regular time period.
     * it returns the stamped final state.
     * @param action_duration the duration of one action
     */
    //4- FINALSTATE MLTIPLEACTIONS STAMPED SYNC
    void predictState(const Stamped<StateType>& initial_state, const vector<ActionType>& action_history,
                      Duration action_duration, Stamped<StateType>& final_state) const;

    /*!
     * @brief Just apply the transfer function multiple times according the given an asynchronous sequence of actions
     * it returns the stamped final state.
     * @param action_history list of asynchronous action. The first action stamp should be the same than the initial_state stamp
     * @param end_time the final prediction time of the simulation. This should be older than the last action stamp.
     */
    //5- FINALSTASTE MULTIPLEACTION STAMPED ASYNC
    template<typename StampedAction>
      void predictState(const Stamped<StateType>& initial_state, const vector<StampedAction>& action_history,
                        TTime end_time, Stamped<StateType>& final_state) const;

    //=========== TRAJECTORY SAMPLING ============

    //6 - SAMPLE SINGLE-ACTION NONSTAMPED SYNC
    void sampleStates(const StateType& initial_state, const ActionType& action, const Duration simulation_duration,
                      StateType* substates, unsigned long sub_states_size) const;

    void sampleStates(const StateType& initial_state, const ActionType& action, const Duration simulation_duration,
                      vector<StateType>& substates) const;

    /*!
     * @fn virtual void sampleStates(const Stamped<StateType, TTime>& initial_state,
     const vector<ActionStep<ActionType> >& action_history, StateType& final_state)
     * It predicts the future state of the robot given an input Action. Abstract.
     * @param initial_state the initial state of the robot with a time stamp
     * @param action_history is a vector of stamped actions that will be applied to the robot initial state estimation.
     * * that the first action stamp should be earlier than the initial_state stamp.
     * @param substates the output param, it is the output of the function. It contains the sampled trajectory.
     */
    //7 - SAMPLE SINGLE-ACTION STAMPED SYNC
  protected:
    template<typename StampedState>
      void sampleStates(const Stamped<StateType>& initial_state, const ActionType& action,
                        const Duration simulation_duration, StampedState* substates,
                        unsigned long sub_states_size) const;

  public:
    template<typename StampedState>
      void sampleStates(const Stamped<StateType>& initial_state, const ActionType& action,
                        const Duration simulation_duration, vector<StampedState>& substates) const;

    //8 - SAMPLE MULTIPLEACTIONS NONSTAMPED SYNC
    void sampleStates(const StateType& initial_state, const vector<ActionType>& action_history,
                      const Duration simulation_duration, vector<StateType>& substates) const;

    //9 - SAMPLE MULTIPLEACTIONS STAMPED SYNC
    template<typename StampedState>
      void sampleStates(const Stamped<StateType>& initial_state, const vector<ActionType>& action_history,
                        Duration simulation_duration, vector<StampedState>& trajectory) const;

    //10 - SAMPLE MULTIPLEACTIONS STAMPED ASYNC
    template<typename StampedState, typename StampedAction>
      void sampleStates(const Stamped<StateType>& initial_state, const vector<StampedAction>& action_history,
                        const TTime& end_time, vector<StampedState>& trajectory) const;

    virtual ~MotionModel()
    {
    }
    ;
  }
  ;
//==================================== TYPICAL DERIVED MOTION MODELS =====================================
/*\brief a motion model where the key is to define the predictLocalStateTransition. It only depends on the actions and not the previous state.
 * q_{t+1} =q_t + B(ut)*/
template<typename StateType, typename ActionType, typename Duration = ros::Duration, typename TTime = ros::Time>
  class KinematicMotionModel : public MotionModel<StateType, ActionType, Duration, TTime>
  {
  protected:
    virtual void transfer_function(const StateType& initial_state, const ActionType& action, const Duration dt,
                                   StateType& final_state) const;

    virtual void predictLocalStateTransition(const ActionType& action, const Duration dt,
                                             StateType& final_state) const=0;
    virtual void composeStates(const StateType& q_a, const StateType& q_b, StateType& q_res) const;

  public:
    virtual ~KinematicMotionModel();
  };

template<typename ProbabilisticStateType, typename DetrministicStateType, typename ActionType,
    typename Duration = ros::Duration, typename TTime = ros::Time>
  class ProbabilisticMotionModel : public MotionModel<ProbabilisticStateType, ActionType, Duration, TTime>
  {
  public:
    //virtual void sampleTrajectory(const ProbabilisticStateType& initial_state, const vector<ActionStep>& action_history,)
  };
}

#endif

