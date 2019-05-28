/*
 * default_state_estimation.h
 *
 *  Created on: Jul 5, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef DEFAULT_STATE_ESTIMATION_H_
#define DEFAULT_STATE_ESTIMATION_H_

#include <rtcus_navigation/state_estimation/default_state_estimation.h>

namespace rtcus_navigation
{
namespace state_estimation
{

template<typename StateType, typename ActionType, typename TimeModel>
  DefaultStateEstimation<StateType, ActionType, TimeModel>::~DefaultStateEstimation()
  {
  }

template<typename StateType, typename ActionType, typename TimeModel>
  DefaultStateEstimation<StateType, ActionType, TimeModel>::DefaultStateEstimation() :
      max_states_(1000), max_actions_(1000), max_states_in_seconds_(5), max_actions_in_seconds_(5), state_readings_history_(
          0), action_buffer_(0)
  {

  }

template<typename StateType, typename ActionType, typename TimeModel>
  void DefaultStateEstimation<StateType, ActionType, TimeModel>::init(
      const rtcus_navigation::AbstractNavigationNode& navigation_node, const ActionType& initialAction)
  //TODO: Remove this "initial_action" Parameter, it is not necessary given the navigation_node parameter
  {
    if (!this->component_node_.getParam("max_state_history_size_in_seconds", max_states_in_seconds_))
      this->component_node_.setParam("max_state_history_size_in_seconds", max_states_in_seconds_);

    if (!this->component_node_.getParam("max_action_buffer_size_in_seconds", max_actions_in_seconds_))
      this->component_node_.setParam("max_action_buffer_size_in_seconds", max_actions_in_seconds_);

    //TODO: declare getters and setters and also use a number proportional to the navigation control frequency
    this->max_states_ = max_states_in_seconds_ * navigation_node.getConfig().planner_frequency;
    this->max_actions_ = max_actions_in_seconds_ * navigation_node.getConfig().planner_frequency;

    ROS_INFO(
        " * State Estimation Module. The initialization time of the state estimation module is: %lf secs. No previous world or goal readings can be taken into account.", ros::Time::now().toSec());

    this->host_node_ = &navigation_node;
    this->initialAction_ = initialAction_;
    RTCUS_ASSERT_MSG(this->motion_model_ != NULL, "A motion model is needed for this state estimation component.");
  }

template<typename StateType, typename ActionType, typename TimeModel>
  bool DefaultStateEstimation<StateType, ActionType, TimeModel>::getLastStateEstimation(
      StampedData<StateType, TTime>& last_state_estimation) const
  {
    if (state_readings_history_.size() == 0)
      return false;
    else
    {
      last_state_estimation = this->last_state_estimation_;
      return true;
    }
  }

template<typename StateType, typename ActionType, typename TimeModel>
  bool DefaultStateEstimation<StateType, ActionType, TimeModel>::getPastState(const TTime& desired_stamp,
                                                                              StateType& past_state) const
  {
    ROS_INFO(" === Predictive State Estimation Module === ");
    ROS_INFO("Getting past state.");
    if (state_readings_history_.size() == 0 || state_readings_history_.front().getStamp() > desired_stamp)
    {
      ROS_ERROR_THROTTLE(
          1.0, "The past state buffer is empty. Or the desired past states is too old and it is not remembered.");
      return false;
    }
    else
    {
      //ROS_INFO("starting getting past state");
      //handle the first element
      StampedStateData past_state_reference = state_readings_history_.front();
      const StampedStateData* past_state_reference_next = NULL;

      //handle the following elements
      StateIterator itprevious = this->state_readings_history_.begin();
      StateIterator itcurrent = this->state_readings_history_.begin();
      itcurrent++;

      //ROS_INFO("looking for the past state in the buffer");
      bool interpolation_needed = true;
      //if it is the first state
      if (past_state_reference.getStamp() == desired_stamp)
      {
        //ROS_INFO("first state exact past state found");
        interpolation_needed = false;

        ROS_INFO(
            " * Past reference state found. Stamped at [%lf] when the desired past state is [%lf]. Notice that this is the first (oldest) state.", past_state_reference.getStamp().toSec(), desired_stamp.toSec());

      }
      //if it is the last state
      else if (state_readings_history_.back().getStamp() < desired_stamp)
      {
        interpolation_needed = true;
        past_state_reference = state_readings_history_.back();
        ROS_INFO(
            " * Past reference state found. Stamped at [%lf] when the desired past state is [%lf]. Notice that this is the last recorded state.", past_state_reference.getStamp().toSec(), desired_stamp.toSec());
      }
      //if it is an intermediate state
      else if (itcurrent != state_readings_history_.end())
      {
        //ROS_INFO("more than one past state, looking in the buffer.");
        while (itprevious != state_readings_history_.end())
        {
          const StampedData<StateType> & previous_state = *itprevious;
          const StampedData<StateType> & current_state = *itcurrent;

          if (previous_state.getStamp() <= desired_stamp && current_state.getStamp() >= desired_stamp)
          {
            //FOUND THE PAST STATE
            //TODO: Linear interpolation and set the right stamp
            past_state_reference = previous_state;
            past_state_reference_next = &current_state;
            interpolation_needed = false;
            ROS_INFO(
                " * Past reference state found. Stamped at [%lf] when the desired past state is [%lf]. Notice that the following recored state is stamped at [%lf]", past_state_reference.getStamp().toSec(), desired_stamp.toSec(), current_state.getStamp().toSec());
            break;
          }
          itprevious++;
          itcurrent++;
        }
      }

      if (!interpolation_needed)
      {
        ROS_INFO(
            "Past state found at (%lf), but it is not exact, requested at (%lf), the next state was (%lf)", past_state_reference.getStamp().toSec(), desired_stamp.toSec(), (past_state_reference_next != NULL) ? past_state_reference_next->getStamp().toSec() : -1.0);
        past_state = past_state_reference.getData();
        const StampedStateData& reference_state_for_prediction = this->getReferenceStateForEstimation(desired_stamp);

        std::vector<ActionDataStamped> selected_actions;
        past_state = *(estimateState(reference_state_for_prediction, desired_stamp, selected_actions));
        //TODO: Here we should make an interpolation between the previous (past_older_state) and the following state
        return true;
      }
      else //we consider that state reading has not already come
      {
        std::vector<ActionDataStamped> selected_actions;
        past_state = this->estimateState(past_state_reference, desired_stamp, selected_actions).getData();
        return true;
      }
    }
  }

template<typename StateType, typename ActionType, typename TimeModel>
  void DefaultStateEstimation<StateType, ActionType, TimeModel>::reset()
  {
    state_readings_history_.clear();
    action_buffer_.clear();

    ros::Time init_time = TTime::now();
    last_state_estimation_ = StampedStateData(init_time, this->host_node_->getReferenceFrame());
  }

template<typename StateType, typename ActionType, typename TimeModel>
  void DefaultStateEstimation<StateType, ActionType, TimeModel>::setMotionModel(
      const boost::shared_ptr<MotionModel<StateType, ActionType, TDuration, TTime> >& motion_model)
  {
    ROS_INFO(
        " * %s. Changing the predictive motion model to %s", getClassName(*this).c_str(), getClassName(*motion_model).c_str());
    this->motion_model_ = motion_model;
  }

template<typename StateType, typename ActionType, typename TimeModel>
  bool DefaultStateEstimation<StateType, ActionType, TimeModel>::getLastCommand(
      Stamped<ActionType, TTime>& last_command) const
  {
    if (action_buffer_.size() > 0)
    {
      last_command = action_buffer_.back();
      return true;
    }
    else
      return false;
  }

template<typename StateType, typename ActionType, typename TimeModel>
  void DefaultStateEstimation<StateType, ActionType, TimeModel>::registerAction(const ActionType& cmd_vel,
                                                                                const TTime& application_time)
  {
    //the recording action mechanism cannot be activated until the first state estimation would be added
    //it is the state_buffer is empty or de action_buffer is empty (both will be filled the first time togheter)
    if (action_buffer_.size() > 0)
    {
//TODO: register both, the current record time and the application time
      action_buffer_.push_back(ActionDataStamped(cmd_vel, application_time, ""));
      if (action_buffer_.size() > (unsigned long)max_actions_)
      {
        action_buffer_.pop_front();
        ROS_WARN("%s State buffer Saturation. removing oldest action records.", getClassName(this).c_str());
      }
    }
  }

template<typename StateType, typename ActionType, typename TimeModel>
  void DefaultStateEstimation<StateType, ActionType, TimeModel>::checkNewEstimationPreconditions(
      const TTime& application_time, const IStateStamped& last_state_reading)
  {
    //if it is the first time add the null action
    if (action_buffer_.size() == 0)
    {
      ROS_WARN(
          "%s - The first state reading has been received. Initializing the stop action in the action history at stamp [%lf]. ", getClassName(*this).c_str(), last_state_reading.getStamp().toSec());
      StampedData<ActionType> null_action(initialAction_, last_state_reading.getStamp(), "");
      action_buffer_.push_front(null_action);
    }

    ROS_INFO("%s - Checking preconditions", getClassName(*this).c_str());
//check preconditions
    {
      ROS_INFO(" |- checking reading state reference frame coherence");
      if (ARE_DISTINCT_FRAMES(this->host_node_->getReferenceFrame(), last_state_reading.getFrameId()))
      {
        RTCUS_ASSERT_MSG(
            false,
            "State Estimation Module. A non-admisible change of the coordinate frame has been made. The state estimation global frame id does not match with the last state correction reading frame id Frames coherence of the last reading state");
      }

      //1. STATE HISTORY UPDATE. Accept newer state notifications of last_state_reading (newer states)
      ROS_INFO(" |- checking reading state stamp coherence");
      if (state_readings_history_.size() == 0
          || last_state_reading.getStamp() > state_readings_history_.back().getStamp())
      {
        state_readings_history_.push_back(
            StampedData<StateType>(*last_state_reading, last_state_reading.getStamp(),
                                   last_state_reading.getFrameId()));
        if (state_readings_history_.size() > (unsigned long)max_states_)
        {
          state_readings_history_.pop_front();
          ROS_WARN_THROTTLE(
              1.0,
              "%s. State buffer Saturation. len ({x}_H) = %ld secs", getClassName(*this).c_str(), this->state_readings_history_.size());
        }
      }
      else
      {
        ROS_WARN(
            "State Estimation module. State reading at stamp (%lf) ignored. using the last state reading (better). State at such stamp already received or it is a unordered older state. Last accepted state reading at stamp (%lf).", last_state_reading.getStamp().toSec(), state_readings_history_.back().getStamp().toSec());
//const StampedStateData& last_state_correction = state_readings_history_.back();
//no return. Ignoring the reading but continue the state estimation
      }
      RTCUS_ASSERT_MSG(action_buffer_.size() > 0, "The action buffer should have at least one action");
      if (application_time < last_state_estimation_.getStamp() || application_time < action_buffer_.back().getStamp())
      {
        RTCUS_ASSERT_MSG(
            application_time > last_state_estimation_.getStamp(),
            "STATE ESTIMATION. The requested application time should be older than the last estimation. This implementation does not support earliers application times since it brokes the incremental state estimation process.");
      }
    }
    ROS_INFO(" |- checking done.");

  }

template<typename StateType, typename ActionType, typename TimeModel>
  void DefaultStateEstimation<StateType, ActionType, TimeModel>::printStateBuffer() const
  {
    int cont2 = 0;
    ROS_INFO("== all states in the buffer ==");
    for (StateIterator it = state_readings_history_.begin(); it != state_readings_history_.end(); it++, cont2++)
    {
      ROS_INFO(
          "State %d at (%lf) [x %lf y %lf phi %lf]", cont2, it->getStamp().toSec(), it->getConstData().pose.x, it->getConstData().pose.y, it->getConstData().pose.phi);
    }
  }

template<typename StateType, typename ActionType, typename TimeModel>
  void DefaultStateEstimation<StateType, ActionType, TimeModel>::push_state_estimation_history(
      const StampedData<StateType, TTime>& estimation)
  {
    PreviousStateEstimation new_estimation;
    TTime now = TTime::now();
    new_estimation.state_estimation = estimation;
    new_estimation.at = now;
    this->estimation_history_.push_back(new_estimation);

    //REMOVE OLD ESTIMATIONS
    if (this->estimation_history_.size() > 0
        && this->estimation_history_.front().at.toSec() < now.toSec() - TDuration(this->max_states_in_seconds_).toSec())
      this->estimation_history_.pop_front();
  }

}
}
#endif /* DEFAULT_STATE_ESTIMATION_H_ */
