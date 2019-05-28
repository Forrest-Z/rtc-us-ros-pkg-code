/*
 * 
 *  Created on: 18/12/2011
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville (Spain) - 2012
 *      License: GPLv3
 */

#ifndef MOTION_MODELS_IMPL_H_
#define MOTION_MODELS_IMPL_H_

#include <rtcus_motion_models/motion_models.h>
#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <ros/ros.h>
#include <rtcus_assert/rtcus_assert.h>

namespace rtcus_motion_models
{

using namespace std;
using namespace rtcus_stamp;
using namespace rtcus_compositions;

inline ros::Duration operator /(ros::Duration a, ros::Duration b)
{
  return ros::Duration(a.toSec() / (float)b.toSec());
}

inline ros::Duration operator /(ros::Duration a, unsigned long b)
{
  return ros::Duration(a.toSec() / (float)b);
}

inline ros::Duration operator +(ros::Duration a, ros::Duration b)
{
  return ros::Duration(a.toSec() + b.toSec());
}

//================================================================================================================================================
// --------------------- Template Implementation -----------------------------------------------------------------------------

//1- FINALSTATE SINGLE-ACTION NONSTAMPED N/A(SYNC)
template<typename StateType, typename ActionType, typename Duration, typename TTime>
  void MotionModel<StateType, ActionType, Duration, TTime>::predictState(const StateType& initial_state,
                                                                         const ActionType& action,
                                                                         const Duration action_duration,
                                                                         StateType& final_state) const
  {
    transfer_function(initial_state, action, action_duration, final_state);
  }

//2- FINALSTATE SINGLE-ACTION STAMPED N/A(SYNC)
template<typename StateType, typename ActionType, typename Duration, typename TTime>
  void MotionModel<StateType, ActionType, Duration, TTime>::predictState(const Stamped<StateType>& initial_state,
                                                                         const ActionType& action,
                                                                         const Duration action_duration,
                                                                         Stamped<StateType>& final_state) const
  {
    ROS_DEBUG("//2- FINALSTATE SINGLE-ACTION STAMPED N/A(SYNC)\n");
    //BOOST_STATIC_ASSERT((boost::is_base_of<Stamped<StateType,TTime>,StampedState >::value));
    this->predictState(initial_state.getConstData(), action, action_duration, final_state.getData());
    final_state.setStamp(initial_state.getStamp() + action_duration);
    final_state.setFrameId(initial_state.getFrameId());
  }

//3- FINALSTATE MLTIPLEACTIONS NONSTAMPED SYNC
template<typename StateType, typename ActionType, typename Duration, typename TTime>
  void MotionModel<StateType, ActionType, Duration, TTime>::predictState(const StateType& initial_state,
                                                                         const vector<ActionType>& action_history,
                                                                         Duration action_duration,
                                                                         StateType& final_state) const
  {
    ROS_DEBUG("//3- FINALSTATE MLTIPLEACTIONS NONSTAMPED SYNC\n");
    StateType& current_state = final_state;
    current_state = initial_state;

    BOOST_FOREACH(const ActionType& action, action_history)
    {
      StateType resulting_state;
      predictState(current_state, action, action_duration, resulting_state);
      current_state = resulting_state;
    }
    final_state = current_state;
  }

//4- FINALSTATE MLTIPLEACTIONS STAMPED SYNC
template<typename StateType, typename ActionType, typename Duration, typename TTime>
  void MotionModel<StateType, ActionType, Duration, TTime>::predictState(const Stamped<StateType>& initial_state,
                                                                         const vector<ActionType>& action_history,
                                                                         Duration action_duration,
                                                                         Stamped<StateType>& final_state) const
  {
    ROS_DEBUG("//4- FINALSTATE MLTIPLEACTIONS STAMPED SYNC\n");
    //BOOST_STATIC_ASSERT((boost::is_base_of<Stamped<StateType,TTime>,StampedState >::value));
    predictState(initial_state.getConstData(), action_history, action_duration, final_state.getData());
    final_state.setStamp(initial_state.getStamp() + action_duration * action_history.size());
    final_state.setFrameId(initial_state.getFrameId());
  }

//5- FINALSTASTE MULTIPLEACTION STAMPED ASYNC
template<typename StateType, typename ActionType, typename Duration, typename TTime>
  template<typename StampedAction>
    void MotionModel<StateType, ActionType, Duration, TTime>::predictState(const Stamped<StateType>& initial_state,
                                                                           const vector<StampedAction>& action_history,
                                                                           TTime end_time,
                                                                           Stamped<StateType>& final_state) const
    {
      ROS_DEBUG("//5- FINALSTASTE MULTIPLEACTION STAMPED ASYNC\n");
      //BOOST_STATIC_ASSERT((boost::is_base_of<Stamped<StateType,TTime>,StampedState >::value));
      BOOST_STATIC_ASSERT((boost::is_base_of<Stamped<ActionType,TTime>,StampedAction >::value));
      RTCUS_ASSERT_MSG(action_history.size() > 0, "The command history should contains at least one element");

      if (action_history[0].getStamp() != initial_state.getStamp())
      {

        ROS_ERROR_THROTTLE(
            1.0,
            "The initial state stamp [%lf sec] should be the same than the first action stamp [%lf sec]", initial_state.getStamp().toSec(), action_history[0].getStamp().toSec());

        RTCUS_ASSERT_MSG(action_history[0].getStamp() == initial_state.getStamp(),
                         "The initial state stamp should be the same that the first action stamp");

      }
      if (action_history.back().getStamp() > end_time)
      {
        ROS_WARN(
            "This not should happen. last action stamp (%lf) > end time stamp (%lf) The final action stamp should be earlier than the end time of the simulation", action_history.back().getStamp().toSec(), end_time.toSec());

      }

      StampedData<StateType> current_state;
      current_state.copyFrom(initial_state);

      Duration action_duration;
      for (unsigned int i = 0; i < action_history.size() - 1; i++)
      {
        StampedData<StateType> resulting_state;
        action_duration = action_history[i + 1].getStamp() - current_state.getStamp();
        predictState(current_state, *(action_history[i]), action_duration, resulting_state);
        current_state = resulting_state;
      }
      //for the final element:
      action_duration = end_time - current_state.getStamp();
      predictState(current_state, *(action_history.back()), action_duration, final_state);
    }

//6 - SAMPLE SINGLE-ACTION NONSTAMPED SYNC
template<typename StateType, typename ActionType, typename Duration, typename TTime>
  void MotionModel<StateType, ActionType, Duration, TTime>::sampleStates(const StateType& initial_state,
                                                                         const ActionType& action,
                                                                         const Duration simulation_duration,
                                                                         StateType* substates,
                                                                         unsigned long samples_count) const

  {
    ROS_DEBUG("//6 - SAMPLE SINGLE-ACTION NONSTAMPED SYNC\n");
    substates[0] = initial_state;
    Duration sampling_dt = simulation_duration / (samples_count - 1);

    Duration current_dt = Duration(0.0);
    for (unsigned long i = 1; i < samples_count; i++)
    {
      current_dt = current_dt + sampling_dt;
      predictState(initial_state, action, current_dt, substates[i]);
      ROS_DEBUG_STREAM(i << substates[i]);
    }
  }

template<typename StateType, typename ActionType, typename Duration, typename TTime>
  void MotionModel<StateType, ActionType, Duration, TTime>::sampleStates(const StateType& initial_state,
                                                                         const ActionType& action,
                                                                         const Duration simulation_duration,
                                                                         vector<StateType>& substates) const
  {
    sampleStates(initial_state, action, simulation_duration, &substates[0], substates.size());
  }

//7 - SAMPLE SINGLE-ACTION STAMPED SYNC
template<typename StateType, typename ActionType, typename Duration, typename TTime>
  template<typename StampedState>
    void MotionModel<StateType, ActionType, Duration, TTime>::sampleStates(const Stamped<StateType>& initial_state,
                                                                           const ActionType& action,
                                                                           const Duration simulation_duration,
                                                                           StampedState* substates,
                                                                           unsigned long samples_count) const
    {
      ROS_DEBUG("//7 - SAMPLE SINGLE-ACTION STAMPED SYNC\n");
      substates[0].copyFrom(initial_state);
      Duration sampling_dt = simulation_duration / (samples_count - 1);
      Duration current_dt = Duration(0.0);

      for (unsigned int i = 1; i < samples_count; i++)
      {
        current_dt += sampling_dt;
        predictState(initial_state, action, current_dt, substates[i]);
        substates[i].setStamp(initial_state.getStamp() + current_dt);
        substates[i].setFrameId(initial_state.getFrameId());
        ROS_DEBUG_STREAM("Sampling substate "<< i <<" "<<*(substates[i]));
      }
      predictState(initial_state, action, simulation_duration, substates[samples_count - 1]);
    }
template<typename StateType, typename ActionType, typename Duration, typename TTime>
  template<typename StampedState>
    void MotionModel<StateType, ActionType, Duration, TTime>::sampleStates(const Stamped<StateType>& initial_state,
                                                                           const ActionType& action,
                                                                           const Duration simulation_duration,
                                                                           vector<StampedState>& substates) const
    {
      ROS_DEBUG("//7 - SAMPLE SINGLE-ACTION STAMPED SYNC\n");
      sampleStates(initial_state, action, simulation_duration, &substates[0], substates.size());
    }

//8 - SAMPLE MULTIPLEACTIONS NONSTAMPED SYNC
template<typename StateType, typename ActionType, typename Duration, typename TTime>
  void MotionModel<StateType, ActionType, Duration, TTime>::sampleStates(const StateType& initial_state,
                                                                         const vector<ActionType>& action_history,
                                                                         Duration simulation_duration,
                                                                         vector<StateType>& trajectory) const
  {
    ROS_DEBUG("//8 - SAMPLE MULTIPLEACTIONS NONSTAMPED SYNC\n");
    RTCUS_ASSERT_MSG(action_history.size() > 0, "The command history should contains at least one element");

    Duration action_duration = simulation_duration / action_history.size();
    Duration sample_duration = simulation_duration / (trajectory.size() - 1);

    StateType current_state = initial_state;
    Duration current_sampling_time = Duration(0);
    TTime current_action_time = TTime(0);
    ROS_DEBUG_STREAM("Initial state"<< current_state);

    unsigned long total_samples_count = 0;
    for (unsigned long action_index = 0; action_index < action_history.size(); action_index++)
    {
      const ActionType& action = action_history[action_index];
      StateType resulting_state;
      //synchronize again the current state with the first sample
      Duration offset = Duration(current_sampling_time.toSec() - current_action_time.toSec());
      if (offset.toSec() > 0)
      {
        predictState(current_state, action, offset, resulting_state);
        current_state = resulting_state;
      }
      unsigned long samples_in_this_action = action_duration.toSec() / sample_duration.toSec();
      Duration total_action_duration_for_sampling = Duration(samples_in_this_action * sample_duration.toSec());
      //next action first sample time reference

      if (total_samples_count + samples_in_this_action >= trajectory.size())
      {
        ROS_DEBUG(
            "Action %ld -> sample index %ld , size %ld, samples_duration: %lf", action_index, total_samples_count, samples_in_this_action, sample_duration.toSec());
        ROS_DEBUG_STREAM(action);
        sampleStates(current_state, action_history[action_index], total_action_duration_for_sampling,
                     &trajectory[total_samples_count], samples_in_this_action);
        ROS_DEBUG_STREAM("Current STATE "<< current_state);
        //current_state = trajectory[total_samples_count + samples_in_this_action - 1];
      }
      else
      {
        ROS_DEBUG(
            "Action %ld -> sample index %ld , size %ld (+1) samples_duration: %lf", action_index, total_samples_count, samples_in_this_action, sample_duration.toSec());
        ROS_DEBUG_STREAM(action);
        ROS_DEBUG_STREAM("Current STATE "<< current_state);
        sampleStates(current_state, action_history[action_index], total_action_duration_for_sampling,
                     &trajectory[total_samples_count], samples_in_this_action + 1);
        //current_state = trajectory[total_samples_count + samples_in_this_action - 1];
        ROS_DEBUG_STREAM("Current STATE "<<trajectory[total_samples_count+samples_in_this_action]);
      }

      total_samples_count += samples_in_this_action;
      predictState(current_state, action, action_duration, resulting_state);
      current_state = resulting_state;
      current_action_time = current_action_time + action_duration;
      current_sampling_time = current_sampling_time + total_action_duration_for_sampling + offset;

    }
    predictState(initial_state, action_history, action_duration, trajectory[trajectory.size() - 1]);
  }

//9 - SAMPLE MULTIPLEACTIONS STAMPED SYNC
template<typename StateType, typename ActionType, typename Duration, typename TTime>
  template<typename StampedState>
    void MotionModel<StateType, ActionType, Duration, TTime>::sampleStates(const Stamped<StateType>& initial_state,
                                                                           const vector<ActionType>& action_history,
                                                                           Duration simulation_duration,
                                                                           vector<StampedState>& trajectory) const
    {
      ROS_DEBUG("//8 - SAMPLE MULTIPLEACTIONS NONSTAMPED SYNC\n");
      RTCUS_ASSERT_MSG(action_history.size() > 0, "The command history should contains at least one element");

      Duration action_duration = simulation_duration / action_history.size();
      Duration sample_duration = simulation_duration / (trajectory.size() - 1);

      StampedData<StateType> current_state;
      current_state.copyFrom(initial_state);

      Duration current_sampling_time = Duration(0);
      TTime current_action_time = TTime(0);

      ROS_DEBUG_STREAM("Initial state"<< *current_state);

      unsigned long total_samples_count = 0;
      for (unsigned long action_index = 0; action_index < action_history.size(); action_index++)
      {
        const ActionType& action = action_history[action_index];
        StampedData<StateType> resulting_state;
        //synchronize again the current state with the first sample
        Duration offset = Duration(current_sampling_time.toSec() - current_action_time.toSec());
        if (offset.toSec() > 0)
        {
          predictState(current_state, action, offset, resulting_state);
          current_state = resulting_state;
        }
        unsigned long samples_in_this_action = action_duration.toSec() / sample_duration.toSec();
        Duration total_action_duration_for_sampling = Duration(samples_in_this_action * sample_duration.toSec());
        //next action first sample time reference

        ROS_DEBUG_STREAM(
            "samples in this action: "<< samples_in_this_action <<", sampling one action duration: " << total_action_duration_for_sampling);

        if (total_samples_count + samples_in_this_action >= trajectory.size())
        {
          ROS_DEBUG(
              "Action %ld -> sample index %ld , size %ld, samples_duration: %lf", action_index, total_samples_count, samples_in_this_action, sample_duration.toSec());
          ROS_DEBUG_STREAM(action);
          sampleStates(current_state, action_history[action_index], total_action_duration_for_sampling,
                       &trajectory[total_samples_count], samples_in_this_action);
          ROS_DEBUG_STREAM("Current STATE "<< *current_state);
          //current_state = trajectory[total_samples_count + samples_in_this_action - 1];
        }
        else
        {
          ROS_DEBUG(
              "Action %ld -> sample index %ld , size %ld (+1) samples_duration: %lf", action_index, total_samples_count, samples_in_this_action, sample_duration.toSec());
          ROS_DEBUG_STREAM(action);
          ROS_DEBUG_STREAM("Current STATE "<< *current_state);
          sampleStates(current_state, action_history[action_index], total_action_duration_for_sampling,
                       &trajectory[total_samples_count], samples_in_this_action + 1);
          //current_state = trajectory[total_samples_count + samples_in_this_action - 1];
          ROS_DEBUG_STREAM("Current STATE "<<*(trajectory[total_samples_count+samples_in_this_action]));
        }

        total_samples_count += samples_in_this_action;
        predictState(current_state, action, action_duration, resulting_state);
        current_state = resulting_state;
        current_action_time = current_action_time + action_duration;
        current_sampling_time = current_sampling_time + total_action_duration_for_sampling + offset;

      }
      predictState(initial_state, action_history, action_duration, trajectory[trajectory.size() - 1]);

    }

//10 - SAMPLE MULTIPLEACTIONS STAMPED ASYNC
template<typename StateType, typename ActionType, typename Duration, typename TTime>
  template<typename StampedState, typename StampedAction>
    void MotionModel<StateType, ActionType, Duration, TTime>::sampleStates(const Stamped<StateType>& initial_state,
                                                                           const vector<StampedAction>& action_history,
                                                                           const TTime& end_time,
                                                                           vector<StampedState>& trajectory) const
    {
      RTCUS_ASSERT(end_time > initial_state.getStamp());
      RTCUS_ASSERT_MSG(action_history.size() > 0, "The command history should contains at least one element");
      RTCUS_ASSERT_MSG(action_history[0].getStamp() == initial_state.getStamp(),
                       "The initial state stamp should be the same that the first action stamp");
      RTCUS_ASSERT(action_history.back().getStamp() < end_time);

      Duration sample_duration = (end_time - initial_state.getStamp()) / (trajectory.size() - 1);
      ROS_DEBUG(
          " end time %lf, initial time %lf, sample duration %lf", end_time.toSec(), initial_state.getStamp().toSec(), sample_duration.toSec());

      StampedData<StateType> current_state;
      current_state.copyFrom(initial_state);
      Duration current_sampling_time = Duration(0);

      unsigned long total_samples_count = 0;
      for (unsigned long action_index = 0; action_index < action_history.size(); action_index++)
      {
        const StampedAction& action = action_history[action_index];
        Duration action_duration;

        if (action_index < action_history.size() - 1)
          action_duration = action_history[action_index + 1].getStamp() - action.getStamp();
        else
          action_duration = end_time - action.getStamp();

        StampedData<StateType> resulting_state;
        //synchronize again the current state with the first sample
        Duration offset = Duration(current_sampling_time.toSec() - action.getStamp().toSec());
        if (offset.toSec() > 0)
        {
          predictState(current_state, *action, offset, resulting_state);
          current_state = resulting_state;
        }
        unsigned long samples_in_this_action = action_duration.toSec() / sample_duration.toSec();
        Duration total_action_duration_for_sampling = Duration(samples_in_this_action * sample_duration.toSec());

        if (total_samples_count + samples_in_this_action >= trajectory.size())
        {
          ROS_DEBUG(
              "Action %ld -> sample index %ld , samples in this action %ld, samples_duration: %lf", action_index, total_samples_count, samples_in_this_action, sample_duration.toSec());
          ROS_DEBUG_STREAM(*action);
          sampleStates(current_state, *action, total_action_duration_for_sampling, &trajectory[total_samples_count],
                       samples_in_this_action);
          ROS_DEBUG_STREAM("Current STATE "<< *current_state);
          //current_state = trajectory[total_samples_count + samples_in_this_action - 1];
        }
        else
        {
          ROS_DEBUG(
              "Action %ld -> sample index %ld , size %ld (+1) samples_duration: %lf", action_index, total_samples_count, samples_in_this_action, sample_duration.toSec());
          ROS_DEBUG_STREAM(*action);
          ROS_DEBUG_STREAM("Current STATE "<< *current_state);
          sampleStates(current_state, *action, total_action_duration_for_sampling, &trajectory[total_samples_count],
                       samples_in_this_action + 1);
          //current_state = trajectory[total_samples_count + samples_in_this_action - 1];
          ROS_DEBUG_STREAM("Current STATE "<<*(trajectory[total_samples_count+samples_in_this_action]));
        }

        total_samples_count += samples_in_this_action;
        predictState(current_state, *action, action_duration, resulting_state);
        current_state = resulting_state;
        //next action first sample time reference
        current_sampling_time = current_sampling_time + total_action_duration_for_sampling + offset;

      }

      predictState(initial_state, action_history, end_time, trajectory[trajectory.size() - 1]);
    }

template<typename StateType, typename ActionType, typename Duration, typename TTime>
  void KinematicMotionModel<StateType, ActionType, Duration, TTime>::transfer_function(const StateType& initial_state,
                                                                                       const ActionType& action,
                                                                                       const Duration dt,
                                                                                       StateType& final_state) const
  {
    StateType partialFinalState;
    predictLocalStateTransition(action, dt, partialFinalState);
    composeStates(initial_state, partialFinalState, final_state);
  }

template<typename StateType, typename ActionType, typename Duration, typename TTime>
  void KinematicMotionModel<StateType, ActionType, Duration, TTime>::composeStates(const StateType& q_a,
                                                                                   const StateType& q_b,
                                                                                   StateType& q_res) const
  {
    StateComposer::compose(q_a, q_b, q_res);
  }

template<typename StateType, typename ActionType, typename Duration, typename TTime>
  KinematicMotionModel<StateType, ActionType, Duration, TTime>::~KinematicMotionModel()
  {
  }

}

#endif

