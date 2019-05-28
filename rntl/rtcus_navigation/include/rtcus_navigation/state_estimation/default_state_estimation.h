/*
 * default_state_estimation.h
 *
 *  Created on: Jul 5, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef DEFAULT_STATE_ESTIMATION_IMP_H_
#define DEFAULT_STATE_ESTIMATION_IMP_H_

#include <rtcus_navigation/state_estimation.h>
namespace rtcus_navigation
{
namespace state_estimation
{
/*
 * \brief This class uses a prediction model to estimate the state in the future and in the past. It stores history buffers about
 * old state readings and action applications.
 *
 * \remarks this class is not thread safe. The decision has been made because the class does not use topic subscribers.
 * */
template<typename StateType, typename ActionType, typename TimeModel = ROSTimeModel>
  class DefaultStateEstimation : public StateEstimation<StateType, ActionType, TimeModel>
  {
    USING_TIME_MODEL(TimeModel);
  private:
    typedef Stamped<StateType, TTime> IStateStamped;
    typedef Stamped<ActionType, TTime> IActionStamped;
    typedef StampedData<StateType, TTime> StampedStateData;
    typedef StampedData<ActionType, TTime> ActionDataStamped;
    typedef typename std::list<ActionDataStamped>::iterator ActionIterator;
    typedef typename std::list<ActionDataStamped>::const_iterator ConstActionIterator;
    typedef typename std::list<StampedStateData>::const_iterator StateIterator;

  public:
    virtual ~DefaultStateEstimation();
    DefaultStateEstimation();
    virtual void init(const rtcus_navigation::AbstractNavigationNode& navigation_node, const ActionType& initialAction);
    virtual void reset();

    /**
     * \brief returns the state history {x}_H
     */
    virtual const std::list<StampedStateData>& getStateHistory() const
    {
      return this->state_readings_history_;
    }

    /**
     * \brief returns the action buffer {u}_B
     * */
    const list<ActionDataStamped>& getNonConfirmedActions() const
    {
      return this->action_buffer_;
    }

    /**
     * \brief gets the just previous state in the  buffer to the given parameter "desired_estimation_stamp":t
     * That is: first (x in {x}_H / x.stamp < t and not any (y in {x}_H / y.stamp>x.stamp and y.stamp <t))
     *
     * It is typically used to understand where were local obstacles at ancient readings used in the method PTC.
     * In any case there is still a futher refination interpolating x_k and x_{k+1} at t
     * */
    const StampedStateData& getReferenceStateForEstimation(const TTime& desired_estimation_stamp) const
    {
      typedef typename std::list<StampedStateData>::const_reverse_iterator RStateIterator;
      //catch the latest previous stored state
      RStateIterator first_non_valid_state = (std::find_if(
          state_readings_history_.rbegin(), state_readings_history_.rend(),
          bind(std::less_equal<TTime>(), bind(&StampedStateData::getStamp, _1), desired_estimation_stamp)));

      RTCUS_ASSERT_MSG(first_non_valid_state != state_readings_history_.rend(),
                       "Desired application time stamp (%lf) but no proper reference state for estimation found",
                       desired_estimation_stamp.toSec());

      return *first_non_valid_state;
    }

    /**
     * \brief refination of the the method getLastStateEstimation. It takes the interpolated state at the given time.
     * It this state it is used as initial state to make the prediction based on the actionbuffer {u}_B
     * */
    virtual StampedStateData estimateState(const StampedStateData& starting_state, const TTime& application_time,
                                           std::vector<ActionDataStamped>& selected_actions) const
    {
      //CHECK IF PREDICTION IS NOT NEEDED
      if (application_time == starting_state.getStamp())
        return starting_state;

      //CHECK IF PREDICTION IS POSSIBLE
      if (!(starting_state.getStamp() >= action_buffer_.front().getStamp()))
      {
        printActionBuffer(this->action_buffer_);
        printActionBuffer(selected_actions);
        this->printStateBuffer();
        // coarse precondition check:  to make a prediction at least one action is needed
        RTCUS_ASSERT_MSG(
            starting_state.getStamp() >= action_buffer_.front().getStamp(),
            "State Estimation Module internal error. At least, the oldest first recorded action [%lf] should be prior or equal to the starting state reference [%lf]",
            action_buffer_.front().getStamp().toSec(), starting_state.getStamp().toSec());
      }

      this->selectActionsForStateEstimation(starting_state.getStamp(), application_time, selected_actions);
      StampedStateData resulting_estimation;
      if (selected_actions.size() == 0 && starting_state.getStamp() == application_time)
      {
        resulting_estimation = starting_state;
      }
      else if (selected_actions.size() >= 1)
      {
        selected_actions.front().setStamp(starting_state.getStamp());
        ROS_INFO("Using motion model: estimate state.");
        this->motion_model_->predictState(starting_state, selected_actions, application_time, resulting_estimation);
        /*this->motion_model_->sampleStates(starting_state, selected_actions_, application_time, trajectory,
         TDuration((application_time - starting_state.getStamp()).toSec() / 10.0));*/

        //resulting_estimation = trajectory.back();
      }
      else if (selected_actions.size() == 0)
      {
        ROS_INFO(
            "last state correction stamp [%lf] and application time is [%lf] and the last action is [%lf]", starting_state.getStamp().toSec(), application_time.toSec(), action_buffer_.back().getStamp().toSec());

        printActionBuffer(this->action_buffer_);
        RTCUS_ASSERT_MSG(selected_actions.size() >= 1,
                         "actions selected to re-estimate the state must be greater than one.");
        RTCUS_ASSERT_MSG(false, "Prediction is not possible without actions");
      }

      RTCUS_ASSERT_MSG(
          resulting_estimation.getStamp() == application_time,
          "check the coherence of this code. This should not happen. The last estimation should be just updated.");

      return resulting_estimation;
    }

    /**
     * \brief Get the state estimation stamped.
     * \brief That is not an estimation for right now. It's an estimation generated during the last computeEstimation
     * call. This estimation may not even be stamped at the past. May be stamped for the future in the case
     * the whole system is working is configured for that (for instance the PTC method implemented in the navigation node computes actions
     * for the future application time x(t_u) taking into account delays in comunication and mechanical actuation)
     * */
    virtual bool getLastStateEstimation(StampedData<StateType, TTime>& last_state_estimation) const;

    virtual bool getPastState(const TTime& desired_stamp, StateType& past_state) const;

    virtual StampedStateData computeStateEstimation(const TTime& application_time,
                                                    const IStateStamped& last_state_reading)
    {
      ROS_INFO("State Estimation Module. Computing new estimation");
      RTCUS_ASSERT_MSG(last_state_reading.getStamp() <= application_time,
                       "State estimation. Get actions to extrapolate from [%lf] to [%lf]. ",
                       last_state_reading.getStamp().toSec(), application_time.toSec());

      this->checkNewEstimationPreconditions(application_time, last_state_reading);
      //1 - GET NEAREST STATE
      const StampedStateData& reference_state_for_prediction = this->getReferenceStateForEstimation(application_time);

      //2 - ESTIMATE STATE
      if (reference_state_for_prediction.getStamp() == application_time)
        this->last_state_estimation_ = reference_state_for_prediction;
      else
      {
        std::vector<ActionDataStamped> selected_actions;
        this->last_state_estimation_ = estimateState(reference_state_for_prediction, application_time,
                                                     selected_actions);
      }

      //3 - CLEAR OLD ACTIONS
      if (action_buffer_.size() > (unsigned long)max_actions_)
        action_buffer_.pop_front();

      //check data coherence
      RTCUS_ASSERT_MSG(
          last_state_estimation_.getStamp() == application_time,
          "State Estimation module. check the coherence of this code. This should not happen. The last estimation should be just updated.");

      this->push_state_estimation_history(last_state_estimation_);
      return last_state_estimation_;
    }

    /**
     * \brief This method is called from the navigation node after a command action is sent. It is necesary for the state prediction.
     * */
    virtual void registerAction(const ActionType& cmd_vel, const TTime& application_time);

    //------------------------------------ SECONDARY METHODS ------------------------------------------------------------------------------------
    virtual bool getLastCommand(Stamped<ActionType, TTime>& last_command) const;

    virtual void setMotionModel(
        const boost::shared_ptr<MotionModel<StateType, ActionType, TDuration, TTime> >& motion_model);

    virtual boost::shared_ptr<MotionModel<StateType, ActionType, TDuration, TTime> > getMotionModel() const
    {
      return this->motion_model_;
    }

  protected:

    virtual void selectActionsForStateEstimation(const TTime& starting_state_stamp, const TTime& application_time,
                                                 std::vector<ActionDataStamped>& selected_actions) const
    {
      RTCUS_ASSERT_MSG(starting_state_stamp < application_time,
                       "State estimation. Get actions to extrapolate from [%lf] to [%lf]. ",
                       starting_state_stamp.toSec(), application_time.toSec());

      ConstActionIterator first_applied_action;
      ConstActionIterator first_non_applied_action;
      {
        first_applied_action = std::find_if(
            this->action_buffer_.begin(), this->action_buffer_.end(),
            bind(std::greater<TTime>(), bind(&ActionDataStamped::getStamp, _1), starting_state_stamp));
        first_applied_action--;

        if (!(first_applied_action->getStamp() <= application_time
            && first_applied_action->getStamp() <= starting_state_stamp))
        {
          ROS_ERROR(
              "State Estimation Module internal error. Making prediction. The first action to be applied is not correct concerning the starting state stamp. starting state [%lf], application time [%lf], first action [%lf]", starting_state_stamp.toSec(), application_time.toSec(), first_applied_action->getStamp().toSec());
          RTCUS_ASSERT_MSG(
              false,
              "State Estimation Module internal error. Making prediction. The first action to be applied is not correct concerning the starting state stamp.");
        }

        first_non_applied_action = (std::find_if(
            this->action_buffer_.begin(), this->action_buffer_.end(),
            bind(std::greater_equal<TTime>(), bind(&ActionDataStamped::getStamp, _1), application_time)));

        if (first_non_applied_action != action_buffer_.end() && first_non_applied_action->getStamp() < application_time)
        {
          RTCUS_ASSERT_MSG(false, "bad search of the first non applied action");
        }

        //if more than one action
        if (first_non_applied_action != action_buffer_.begin())
        {
          ConstActionIterator last_applied_action = first_non_applied_action;
          last_applied_action--;
          if (last_applied_action != first_applied_action)
          {
            bool error = false;
            if (last_applied_action->getStamp() >= application_time)
            {
              printActionBuffer(this->action_buffer_);
              printStateBuffer();
              error = true;
              ROS_ERROR(
                  "State estimation. Get actions to extrapolate from [%lf] to [%lf]. The last action at [%lf] it is not lesser and equal than the application time of the end state at [%lf]. ", starting_state_stamp.toSec(), application_time.toSec(), last_applied_action->getStamp().toSec(), application_time.toSec());

            }

            if (first_applied_action->getStamp() > starting_state_stamp)
            {
              printActionBuffer(this->action_buffer_);
              printStateBuffer();
              error = true;
              ROS_ERROR(
                  "State estimation. Get actions to extrapolate from [%lf] to [%lf]. The first action at [%lf] is greater than the reference start state stamp [%lf]. ", starting_state_stamp.toSec(), application_time.toSec(), first_applied_action->getStamp().toSec(), starting_state_stamp.toSec());
            }

            RTCUS_ASSERT(!error);
          }
        }
      }

      //selecting actions
      //TODO: Optimize this to avoid calling allocation
      selected_actions.reserve(max_actions_);
      {
        if (first_non_applied_action != first_applied_action)
        {
          int cont = 0;
          for (ConstActionIterator it = first_applied_action;
              it != first_non_applied_action && it != action_buffer_.end(); it++, cont++)
            selected_actions.push_back(*it);

          selected_actions.resize(cont);

        }
        else
        {
          printActionBuffer(this->action_buffer_);
          printStateBuffer();
          ROS_ERROR(
              "%s. Get actions to extrapolate from [%lf] to [%lf].", getClassName(*this).c_str(), starting_state_stamp.toSec(), application_time.toSec());
          ROS_ERROR("Estimation .No action applicable.Imposible situation?");

          RTCUS_ASSERT(false);
        }
      }
    }

    virtual void checkNewEstimationPreconditions(const TTime& application_time,
                                                 const IStateStamped& last_state_reading);

    //======================= FOR DEBUGGING ===================================================================
    void printActionBuffer(const std::vector<ActionDataStamped>& selected_actions) const
    {
      int cont2 = 0;
      ROS_INFO("== all actions in the buffer ==");
      typedef typename std::vector<ActionDataStamped>::const_iterator vConstActionIterator;
      for (vConstActionIterator it = selected_actions.begin(); it != selected_actions.end(); it++, cont2++)
      {
        ROS_INFO(
            "Action %d at (%lf) [v %lf omega %lf]", cont2, it->getStamp().toSec(), it->getConstData().linear, it->getConstData().angular);

      }
    }

    void printActionBuffer(const std::list<ActionDataStamped>& actions) const
    {
      int cont2 = 0;
      ROS_INFO("== all actions in the buffer ==");
      for (ConstActionIterator it = actions.begin(); it != actions.end(); it++, cont2++)
      {
        ROS_INFO(
            "Action %d at (%lf) [v %lf omega %lf]", cont2, it->getStamp().toSec(), it->getConstData().linear, it->getConstData().angular);

      }
    }
    void printStateBuffer() const;

  protected:
    //FIELDS
    boost::shared_ptr<MotionModel<StateType, ActionType, TDuration, TTime> > motion_model_;
    unsigned int max_states_;
    unsigned int max_actions_;
    double max_states_in_seconds_;
    double max_actions_in_seconds_;
    ActionType initialAction_;
    const AbstractNavigationNode* host_node_;

    /**
     * \brief last ^x(t_u)
     * */
    StampedData<StateType, TTime> last_state_estimation_;

    /***
     * \brief this class match the state history concept {x}_H in the PTC algorithm
     * */
    std::list<StampedStateData> state_readings_history_;

    /***
     * \brief this class match the state history concept {u}_B in the PTC algorithm
     * */
    std::list<ActionDataStamped> action_buffer_;

    void push_state_estimation_history(const StampedData<StateType, TTime>& estimation);

  public:
    /**
     * \brief helper struct to store the history of estimations
     * */

    struct PreviousStateEstimation
    {
      StampedStateData state_estimation;

      /*When the prediction was done*/
      TTime at;

    };
  protected:
    std::list<PreviousStateEstimation> estimation_history_;

  public:
    const std::list<PreviousStateEstimation>& getStateEstimationHistory() const
    {
      return estimation_history_;
    }

  }
  ;

}
}
#endif /* DEFAULT_STATE_ESTIMATION_H_ */
