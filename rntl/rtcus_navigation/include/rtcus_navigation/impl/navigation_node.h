/*
 *
 *  Created on: Apr 6, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef NAVIGATION_NODE_IMP_H_
#define NAVIGATION_NODE_IMP_H_

#include <rtcus_navigation/navigation_node.h>
#include <rtcus_navigation/task_status.h>
#include<rtcus_navigation/collision_checkers/covariant_collision_checker.h>
#include <rtcus_navigation/impl/navigationnode_component_dynamic_load.h>
#include<rtcus_navigation/collision_checkers/covariant_collision_checker.h>

#include <boost/type_traits.hpp>
#include <rtcus_assert/rtcus_assert.h>
#include <vector>
#include <boost/algorithm/string.hpp>

namespace rtcus_navigation
{

using namespace std;
using namespace rtcus_navigation;
using namespace boost;
using namespace rtcus_stamp;

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::NavigationNode() :
      state_estimation_pub_(this->private_node_, getComponentTypeName(rtcus_navigation::tStateEstimation))
  {

  }

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  void NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::onInit()
  {
    ros::NodeHandle& nh = this->node_;

    //===== DYNAMIC PLUGINLIB COMPONENT LOADS ========================
    string component_type_name;
    ROS_INFO("== Loading Dynamically Specified Components ==");
    if (this->state_estimation_ == NULL
        && this->private_node_.getParam(getComponentTypeName(tStateEstimation) + "_type", component_type_name))
    {
      this->loadStateEstimation(component_type_name);
    }

    if (this->action_port_ == NULL
        && this->private_node_.getParam(getComponentTypeName(tActionPort) + "_type", component_type_name))
    {
      this->loadActionPort(component_type_name);
    }

    if (this->state_correction_port_ == NULL
        && this->private_node_.getParam(getComponentTypeName(tStatePort) + "_type", component_type_name))
    {
      this->loadStatePort(component_type_name);
    }

    ROS_INFO("=== Checking Navigation Architecture Components==");
    bool error = false;
    //TODO: THIS CAN BE DONE NOW PROGRAMATICALLY

    /*
     int i=0;
     BOOST_FOREACH(std::string comp_name, __navComponentsNames)
     {
     ROS_INFO_STREAM(" * Cheking - "comp_name<< " - ...");

     if (comp == NULL)
     {
     error = true;
     ROS_ERROR("[NOT SPECIFIED]");
     }
     i++;
     }*/

    ROS_INFO("navigation planner ...");
    if (this->navigation_planner_ == NULL)
    {
      error = true;
      ROS_ERROR("[NOT SPECIFIED]");
    }

    ROS_INFO("world perception port ...");
    if (this->perception_port_ == NULL)
    {
      error = true;
      ROS_ERROR("[NOT SPECIFIED]");
    }

    ROS_INFO("state correction port ...");
    if (this->state_correction_port_ == NULL)
    {
      error = true;
      ROS_ERROR("[NOT SPECIFIED]");
    }

    ROS_INFO("action port ...");
    if (this->action_port_ == NULL)
    {
      error = true;
      ROS_ERROR("[NOT SPECIFIED]");
    }

    ROS_INFO("goal port ...");
    if (this->goal_port_ == NULL)
    {
      error = true;
      ROS_ERROR("[NOT SPECIFIED]");
    }

    ROS_INFO("state estimation ...");
    if (this->state_estimation_ == NULL)
    {
      error = true;
      ROS_ERROR("[NOT SPECIFIED]");
    }

    ROS_INFO("reach goal ...");
    if (this->reached_goal_predicate_ == NULL)
    {
      error = true;
      ROS_ERROR("[NOT SPECIFIED]");
    }

    ROS_INFO("collision checking ...");
    if (this->collision_checker_ == NULL)
    {
      error = true;
      ROS_ERROR("[NOT SPECIFIED]");
    }

    ROS_INFO("kinodynamic model checking ...");
    if (this->kinodynamic_model_ == NULL)
    {
      error = true;
      ROS_ERROR("[NOT SPECIFIED]");
    }

    ROS_INFO("robot shape checking ...");
    if (this->robot_shape_model_ == NULL)
    {
      error = true;
      ROS_ERROR("[NOT SPECIFIED]");
    }

    if (error)
    {
      ROS_FATAL("All components of the distributed architecture were not available..");
      exit(-1);
    }
    ROS_INFO("=======");

    //initializing fields
    last_goal_reached_detected = false;
    last_prediction_goal_reached_detected = false;

    ROS_INFO(" * Subscribing to the commands topic...");
    this->command_sub_ = nh.subscribe<rtcus_navigation::ControlCommand>(
        "control_command",
        10,
        bind(
            &NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription,
                TimeModel>::control_command_callback,
            this, _1));
    ROS_INFO(" * Publishing status topic...");
    this->status_pub_ = nh.advertise<rtcus_navigation::Status>("status", 10, true);
    ROS_INFO(" * Publishing status prediction topic...");
    this->status_prediction_pub_ = nh.advertise<rtcus_navigation::StatusPrediction>("status_prediction", 10, true);

    //--------- init components ---------------------
    ROS_INFO("=== Initializing Navigation Architecture Components ===");
    ROS_INFO(" * World perception port...%s", this->perception_port_->type_name().c_str());
    this->perception_port_->init();

    ROS_INFO(" * Action port...%s", this->action_port_->type_name().c_str());

    this->action_port_->init();
    ROS_INFO(" * Goal port...%s", this->goal_port_->type_name().c_str());

    this->goal_port_->init(this->getReferenceFrame(), this->getBaseFrameName());

    ROS_INFO(" * State estimation component...%s", this->state_estimation_->type_name().c_str());
    {
      //TODO: This action stop action should be constrained by the kinodynamic model
      //for now it is free, but it shouldn't. But for now it is better to maintain the generality of the framework
      ActionType stopAction;
      action_port_->getStopAction(stopAction);
      this->state_estimation_->init(*this, stopAction);
    }
    ROS_INFO(" * State port...%s", this->state_correction_port_->type_name().c_str());
    this->state_correction_port_->init(this->getReferenceFrame());

    ROS_INFO(" * Reach goal predication...%s", this->reached_goal_predicate_->type_name().c_str());
    this->reached_goal_predicate_->init();

    ROS_INFO(" * RobotShapeModel...");
    this->robot_shape_model_->init();

    ROS_INFO(" * Kinodynamic Model: %s", this->robot_shape_model_->type_name().c_str());
    this->kinodynamic_model_->init();

    ROS_INFO(" * Collision checking... %s", this->collision_checker_->type_name().c_str());
    this->collision_checker_->init();

    ROS_INFO(" * Navigation planner...%s", this->navigation_planner_->type_name().c_str());
    this->navigation_planner_->init(*this);
    ROS_INFO("======");

    this->perception_port_->allocateObstacleRepresentation(this->current_status_.resulting_obstacles_);
    //this->state_correction_port->allocateStateRepresentation(this->current_status_.state_estimation_);

  }

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  void NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::onConfigurationUpdate(
      NavigationArchitectureConfig &config)
  {
    if (this->config_.state_estimation_type != config.state_estimation_type)
    {
      ROS_INFO("* New state estimation module requested: %s", config.state_estimation_type.c_str());
      if (this->loadStateEstimation(config.state_estimation_type))
      {
        //accepted
        this->config_.state_estimation_type = config.state_estimation_type;
        ActionType initial_action;
        action_port_->getStopAction(initial_action);
        this->state_estimation_->init(*this, initial_action);
      }
      else
      {
        config.state_estimation_type = this->config_.state_estimation_type;
      }
    }
    this->onConfigurationUpdated(*this);
  }

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  void NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::onReset()
  {
    current_status_ = TTaskStatus();
    this->perception_port_->allocateObstacleRepresentation(this->current_status_.resulting_obstacles_);

    this->navigation_planner_->reset();
    this->perception_port_->reset();
    this->goal_port_->reset();
    this->state_estimation_->reset();
    this->state_correction_port_->reset();
    this->action_port_->reset();
    this->reached_goal_predicate_->reset();
    this->collision_checker_->reset();

    last_goal_reached_detected = false;
    last_prediction_goal_reached_detected = false;
  }

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  void NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::control_command_callback(
      const rtcus_navigation::ControlCommand::ConstPtr& msg)
  {
    boost::mutex::scoped_lock l(m_mutex);
    std::string cmd = msg->command_value;
    boost::to_upper(cmd);
    ROS_INFO("Navigation Node. Control command received: %s", cmd.c_str());

    if (cmd == rtcus_navigation::ControlCommand::STOP_COMMAND)
    {
      ROS_INFO("control command STOP");
      TTime application_time = TTime::now() + this->getActionPort()->getExpectedTimeDelay();
      ActionType stopAction;
      this->action_port_->getStopAction(stopAction);
      this->action_port_->sendAction(stopAction, TTime::now());
      this->state_estimation_->registerAction(stopAction, application_time);
      this->control_timer_.stop();
    }
    else if (cmd == rtcus_navigation::ControlCommand::START_COMMAND)
    {
      ROS_INFO("control command START");
      this->control_timer_.start();
    }
    else if (cmd == rtcus_navigation::ControlCommand::RESET_NODE)
    {
      ROS_INFO("control command RESET");
      this->control_timer_.stop();
      if (!this->control_timer_.hasPending())
      {
        this->reset();
      }
      else
        ROS_INFO("reset command but there is pending calls. WHAT TO DO?");
    }
    else
      ROS_INFO("not recognized control command");
  }

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::~NavigationNode()
  {

  }
;

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  void NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::controlTask(
      const ros::TimerEvent time)
  {

    boost::mutex::scoped_lock l(m_mutex);
    //TAKE A COPY OF THE GOAL g(t_g) AND THE STATE x(t_x)
    {
      current_status_.planning_time_ = TTime::now();
      TDuration dt_to_application_time = TDuration(this->getActionPort()->getExpectedTimeDelay());
      TTime application_time = current_status_.planning_time_ + dt_to_application_time;

      current_status_.init(application_time);
      state_correction_port_->getLastState(current_status_.last_state_reading_);
      this->goal_port_->getGoalEstimation(application_time, current_status_.last_goal_reading_);

      TDuration delta_t_state_reading = application_time - current_status_.last_state_reading_.getStamp();
      TDuration delta_t_goal_reading = application_time - current_status_.last_goal_reading_.getStamp();
      TDuration known_system_delay = time.current_expected - time.current_real;

      if (delta_t_state_reading < TDuration(0.0) || delta_t_goal_reading < TDuration(0.0))
      {
        ROS_WARN_THROTTLE(
            1.0,
            "Navigation step skipped. The system looks delayed [Known delay %lf]. [application time in %lf (at %lf) last state reading: %lf (%lf), last goal %lf (%lf)]", known_system_delay.toSec(), dt_to_application_time.toSec(), application_time.toSec(), current_status_.last_state_reading_.getStamp().toSec(), delta_t_state_reading.toSec(), current_status_.last_goal_reading_.getStamp().toSec(), delta_t_goal_reading.toSec());
        return;
      }
    }

    onControlTaskBegin(*this, current_status_.getExpectedApplicationTime(), time, current_status_);
    {
      step_estimate_state(current_status_.state_estimation_, current_status_);
      step_estimate_goal(current_status_.resulting_goal_, current_status_.state_estimation_, current_status_);
      step_checking_goal_predicate(current_status_.state_estimation_, current_status_);

      {
        shared_ptr<mutex> obstacle_mutex = this->perception_port_->getWorldEstimation(
            current_status_.getExpectedApplicationTime(), current_status_.last_obstacle_reading_);
        RTCUS_ASSERT_MSG(!obstacle_mutex || !obstacle_mutex->try_lock(), "The mutex has to be given already locked");

        step_process_local_world(current_status_.resulting_obstacles_, current_status_.state_estimation_,
                                 current_status_);

        obstacle_mutex->unlock();
      }

      step_check_collision(current_status_.state_estimation_, current_status_.resulting_obstacles_, current_status_);

      ActionType cmd_vel;
      step_planning(cmd_vel, current_status_.state_estimation_, current_status_.resulting_goal_,
                    current_status_.resulting_obstacles_, current_status_);
      step_action_apply(cmd_vel, current_status_);
      step_publish_status(current_status_);
    }
    onControlTaskEnd(*this, current_status_);
  }

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  void NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::step_estimate_state(
      StampedData<StateType>& state_estimation, TTaskStatus& status)
  {
    ROS_DEBUG("1- STATE ESTIMATION");
    onStateEstimationBegin(*this, status);
    if (!state_correction_port_->hasValidState())
    {
      ROS_WARN(
          " * NavigationNode using the StatePort-> EMERGENCY STOP: There no is a valid state reading so navigation it is not recomendable.");
    }
    else
    {

      if (ARE_DISTINCT_FRAMES(status.last_state_reading_.getFrameId(), this->getReferenceFrame()))
      {
        ROS_ERROR(
            " * NavigationNode using the StatePort-> EMERGENCY STOP: WRONG state prediction frame id provided by the navigation node. State prediction header frame [%s] does not match with frame [%s]", status.last_state_reading_.getFrameId().c_str(), getReferenceFrame().c_str());
      }
      else
      {
        status.valid_state_estimation = true;
        state_estimation = this->state_estimation_->computeStateEstimation(status.application_time_,
                                                                           status.last_state_reading_);
      }
    }
    //state estimation in global coordinates
    state_estimation.setFrameId(getReferenceFrame());
    state_estimation.setStamp(status.application_time_);

//or also
//state_estimation.setFrameId(getBasePredictionFrame());
//state_estimation.setFrameId(TTime::now());

    onStateEstimationEnd(*this, status);
  }

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  void NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::step_estimate_goal(
      StampedData<GoalType>& resulting_goal, StampedData<StateType>& state_estimation, TTaskStatus& status)
  {
    ROS_DEBUG("2- PROCESING LOCAL GOAL");
    //specifies the position of the past local frame in the current local frame
    StateType state_at_goal_notification;
    onProcessingLocalGoalBegin(*this, status);
//CHECK GOAL ESTIMATION PRECONDITIONS
    if (!status.valid_state_estimation)
    {
      ROS_WARN(" * Local goal processing canceled since the state estimation is not valid.");
    }
    else if (ARE_DISTINCT_FRAMES(status.last_goal_reading_.getFrameId(), this->getReferenceFrame())
        && ARE_DISTINCT_FRAMES(status.last_goal_reading_.getFrameId(), this->getBaseFrameName()))
    {
      //NOT ACCEPT OTHER KIND OF REFERENCED GOALS
      ROS_WARN(
          "GOAL PREDICTION ERROR -> EMERGENCY STOP: The goal has not been specified in the local frame [%s] or the global frame [%s]. Instead (%s) frame was used. NAVIGATION CONTROL STEP ABORTED.", this->getBaseFrameName().c_str(), this->getReferenceFrame().c_str(), status.last_goal_reading_.getFrameId().c_str());
    }
    else if ((ARE_SAME_FRAMES(status.last_goal_reading_.getFrameId(), this->getBaseFrameName())
        && state_estimation_->getPastState(status.last_goal_reading_.getStamp(), state_at_goal_notification))
        || ARE_SAME_FRAMES(status.last_goal_reading_.getFrameId(), this->getReferenceFrame()))

    {

      //2.1 - accept GLOBAL-FRAME GOALS
      if (ARE_SAME_FRAMES(status.last_goal_reading_.getFrameId(), this->getReferenceFrame()))
      {
        //If goal is expressed in the global frame, then just change the goal reference frame to the robot local frame.
        StateComposer::inverse_compose(status.last_goal_reading_.getData(), state_estimation.getConstData(),
                                       resulting_goal.getData());
      }
      //2.2 - accept also LOCAL BASE LINK - GOALS
      else //if (ARE_SAME_FRAMES(goal_estimation.getFrameId(), this->getBaseFrameName()))
      {
        StateType delta_transform;
        //Ax = x(t_x) - ^x(^t_u)
        StateComposer::inverse_compose(state_estimation.getData(), state_at_goal_notification, delta_transform);
        //^g(^t_u)= g(t_g) - Ax
        StateComposer::inverse_compose(status.last_goal_reading_.getData(), delta_transform, resulting_goal.getData());
      }
      status.valid_goal_estimation = true;
    }
    else
    {
      ROS_ERROR(
          "GOAL PREDICTION ERROR. Navigation loop step aborted: goal specified in the past but the system does not remember the state at such past time. May you need a latchable goal port?");

    }
    resulting_goal.setFrameId(this->getBasePredictionFrame());
    resulting_goal.setStamp(status.application_time_);
    onProcessingLocalGoalEnd(*this, *resulting_goal, status.application_time_);

  }
template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  void NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::step_checking_goal_predicate(
      StampedData<StateType>& state_estimation, TTaskStatus& status)
  {
    ROS_DEBUG("3 - CHECK GOAL PREDICTION REACH PREDICATE");
    StampedData<GoalType> goal_estimation;
    this->goal_port_->getGoalEstimation(status.application_time_, goal_estimation);

    if (status.valid_state_estimation && status.valid_goal_estimation
        && reached_goal_predicate_->isGoalReached(state_estimation.getData(), goal_estimation.getConstData()))
    {
      status.goal_reached_prediction = true;
      std::stringstream ss;
      if (ros::message_traits::IsMessage<StateType>::value == true)
        ss << "STATE: " << state_estimation.getData();
      if (ros::message_traits::IsMessage<GoalType>::value == true)
        ss << "GOAL" << goal_estimation.getConstData();
      ROS_INFO_STREAM("* REACHED GOAL (PREDICTION)" << ss);

    }
    else
      status.goal_reached_prediction = false;
  }

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  void NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::step_process_local_world(
      Stamped<ObstaclesType>& resulting_obstacles, StampedData<StateType>& state_estimation, TTaskStatus& status)
  {
    ROS_DEBUG("4 - PROCESSING LOCAL WORLD");
    onProcessingLocalWorldBegin(*this, status);
    {
      //PRECONDITIONS
      if (current_status_.last_obstacle_reading_.getStamp() > status.getExpectedApplicationTime())
      {
        ROS_WARN(
            "Obstacles too much new (%lf), even newer than the expected application time (%lf). breaking to the next step. Perhaps the navigation loop is not working at the frequency it should"
            "or the sensor data is saturating the planning loop", current_status_.last_obstacle_reading_.getStamp().toSec(), status.getExpectedApplicationTime().toSec());
      }

      bool global_obstacles_description = ARE_SAME_FRAMES(status.last_obstacle_reading_.getFrameId(),
                                                          this->getReferenceFrame());
      bool local_obstacles_description = ARE_SAME_FRAMES(status.last_obstacle_reading_.getFrameId(),
                                                         this->getBaseFrameName());

      bool obstacle_preconditions = true;

      if (!perception_port_->hasValidObstaclesEstimation())
      {
        ROS_WARN(" * Local world processing. Not yet initialized obstacles (or invalid) -> EMERGENCY STOP.");
        obstacle_preconditions = false;
      }

      if (perception_port_->hasValidObstaclesEstimation() && !global_obstacles_description
          && !local_obstacles_description)
      {
        ROS_WARN_THROTTLE(
            1.0,
            "Navigation Node. Local world processing. Obstacles error-> EMERGENCY STOP. The obstacles has not been specified in the local frame [%s] or the global frame [%s]. Instead they have been specified in the [%s] frame", this->getBaseFrameName().c_str(), this->getReferenceFrame().c_str(), status.last_obstacle_reading_.getFrameId().c_str());
        obstacle_preconditions = false;
      }

      //------------ COMPUTE OBSTACLES -----------------------------------------------------------------
      //Get state at obstacle reading x(t_o)
      //x(t_o) = interpolate ({x}_H) at t_o
      if (perception_port_->hasValidObstaclesEstimation() && local_obstacles_description
          && !this->state_estimation_->getPastState(status.last_obstacle_reading_.getStamp(),
                                                    *status.state_at_obstacle_reading_))
      {

        obstacle_preconditions = false;
        ROS_WARN_THROTTLE(
            1.0,
            "Navigation Node. Local world processing. Obstacle PREDICTION ERROR-> EMERGENCY STOP. Navigation loop step aborted: [%s]. The obstacles are stamped at [%lf]. Too old time and the state is not remembered", status.last_obstacle_reading_.getFrameId().c_str(), status.last_obstacle_reading_.getStamp().toSec());
      }

      if (obstacle_preconditions && status.valid_state_estimation && status.valid_goal_estimation)
      {
        //PROCESSING OBSTACLES
        //case A - Global obstacles processing
        if (global_obstacles_description)
        {
          ROS_INFO_ONCE("USING GLOBAL OBSTACLES DESCRIPTION (This message will be printed once)");
          resulting_obstacles = status.last_obstacle_reading_;
        }
        //case B - Local Obstacles processing
        else if (local_obstacles_description)
        {
          if (status.last_state_reading_.getStamp() < status.last_obstacle_reading_.getStamp())
            ROS_WARN("This estimation could be improved. See original paper.");

          ROS_INFO_ONCE("USING LOCAL OBSTACLES DESCRIPTION (This message will be printed once)");
          StateType delta_transform;
          // epsilon_o = x(t_u) (-) x(t_o)
          StateComposer::inverse_compose(*state_estimation, *(status.state_at_obstacle_reading_), delta_transform);
          // o(t_u) = o(t_o) (-) epsilon_o
          StateComposer::inverse_compose(*(status.last_obstacle_reading_), delta_transform, *resulting_obstacles);

        }
        resulting_obstacles.setFrameId(this->getBasePredictionFrame());
        resulting_obstacles.setStamp(status.application_time_);
        status.obstacles_computed = true;

        //or also
        //resulting_obstacles.setFrameId(this->getBasePredictionFrame());
        //resulting_obstacles.setStamp(TTime::now());
      }
    }
    onProcessingLocalWorldEnd(*this, *resulting_obstacles);

  }

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  void NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::step_check_collision(
      StampedData<StateType>& state_estimation, Stamped<ObstaclesType>& resulting_obstacles, TTaskStatus& status)
  {
    ROS_DEBUG("5 - CHECKING COLLISION PREDICTION");

    //ŝ(t_u)
    this->robot_shape_model_->predictRobotShape(status.application_time_, *status.resulting_shape_);
    status.resulting_shape_.setStamp(status.application_time_);
    status.resulting_shape_.setFrameId(this->getBaseFrameName());

    if (status.valid_state_estimation && status.obstacles_computed
        && collision_checker_->detectCollision(*resulting_obstacles, *status.resulting_shape_))
    {
      ROS_WARN("Collision prediction detected");
      status.collision_prediction = true;
    }
  }

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  void NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::step_planning(
      ActionType& cmd_vel, StampedData<StateType>& state_estimation, StampedData<GoalType>& resulting_goal,
      Stamped<ObstaclesType>& resulting_obstacles, TTaskStatus& status)
  {
    ROS_DEBUG("6 - PLANNING AND COMPUTE NEW COMMAND");
    onPlanningBegin(*this, status);
    if (status.valid_state_estimation && status.valid_goal_estimation && status.obstacles_computed)
    {
      try
      {
        navigation_planner_->computeVelocityCommands(*resulting_obstacles, *resulting_goal, *state_estimation, cmd_vel);
        onPlanningResult(*this, cmd_vel, status.application_time_);
      }
      catch (std::exception& e)
      {
        ROS_WARN("Error in motion planning: %s", e.what());
      }
    }
    else
    {
      ROS_WARN(
          " * Planning canceled since not all required information is available. valid_state_estimation=%d valid_goal_estimation=%d goal_reached_prediction=%d", status.valid_state_estimation, status.valid_goal_estimation, status.goal_reached_prediction);
    }
    onPlaningEnd(*this, status);
  }

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  void NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::step_action_apply(
      ActionType& cmd_vel, TTaskStatus& status)
  {
    ROS_INFO("7 - APPLY ACTION");
    if (status.valid_state_estimation && status.valid_goal_estimation && !status.goal_reached_prediction)
    {
      this->action_port_->sendAction(cmd_vel, status.application_time_);
      this->state_estimation_->registerAction(cmd_vel, status.application_time_);
    }
    else
    {
      //DANGEROUS SITUATION STOP THE ROBOT
      ActionType stopAction;
      this->action_port_->getStopAction(stopAction);
      this->action_port_->sendAction(stopAction, TTime::now());
      this->action_port_->sendAction(stopAction, status.application_time_);

      //TODO: The stop action should be defined in an standard way. For instance using the kinodynamic description
      //of the robot.
      this->state_estimation_->registerAction(stopAction, status.application_time_);
      ROS_WARN(
          " * EMERGENCY STOP COMMAND SENT. Ignoring Planner decision: valid_state_estimation=%d valid_goal_estimation=%d goal_reached_prediction=%d", status.valid_state_estimation, status.valid_goal_estimation, status.goal_reached_prediction);
    }
  }

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  void NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::step_publish_status(
      TTaskStatus& status)
  {
    ROS_DEBUG("8 - PUBLISING PREDICTION INFORMATION STATE");

    StatusPrediction prediction_status_msg;

    //--------------------  COLLISION PREDICTION ---------------------------
    prediction_status_msg.expected_stamp = status.application_time_;
    prediction_status_msg.current_stamp = TTime::now();
    if (status.goal_reached_prediction && !last_prediction_goal_reached_detected)
    {
      prediction_status_msg.status_type.value = StatusType::GOAL_REACHED;
      this->status_prediction_pub_.publish(prediction_status_msg);
      last_prediction_goal_reached_detected = true;
    }
    else if (!status.goal_reached_prediction && last_prediction_goal_reached_detected)
    {
      prediction_status_msg.status_type.value = StatusType::GOAL_LOST;
      this->status_prediction_pub_.publish(prediction_status_msg);
      last_prediction_goal_reached_detected = false;
    }

    if (status.collision_prediction)
    {
      prediction_status_msg.status_type.value = StatusType::COLLISION;
      this->status_prediction_pub_.publish(prediction_status_msg);
    }

    this->state_estimation_pub_.publish(status.state_estimation_);

    //--------------------  GOAL REACHED AND CONFIRMED -------------------------
    Stamped<StateType> &last_state_reading = status.last_state_reading_;
    StampedData<GoalType> goal_estimation;
    this->goal_port_->getGoalEstimation(status.application_time_, goal_estimation);
    bool error = false;

    if (!state_correction_port_->hasValidState())
    {
      ROS_ERROR( "NavigationNode. Checking goal reach. No valid state read");
      error = true;
    }

    if (ARE_DISTINCT_FRAMES(last_state_reading.getFrameId(), this->getReferenceFrame()))
    {
      ROS_ERROR(
          "NavigationNode. Checking goal reach. Last state not valid [%d] or frames do not match: last state reading frame [%s] does not match with the reference frame [%s]", this->state_correction_port_->hasValidState(), last_state_reading.getFrameId().c_str(), getReferenceFrame().c_str());
      error = true;
    }

    if (error)
    {
      ROS_ERROR("NavigationNode. Checking goal reach aborted. No valid state read available.");
      return;
    }
    else if (reached_goal_predicate_->isGoalReached(*last_state_reading, goal_estimation.getConstData()))
    {
      this->onGoalReached(*this, status);
      Status status_msg;
      status_msg.stamp = last_state_reading.getStamp();
      status_msg.status_type.value = StatusType::GOAL_REACHED;
      this->status_pub_.publish(status_msg);
      last_goal_reached_detected = true;
    }
    else if (last_goal_reached_detected)
    {
      this->onGoalLost(*this, status);
      Status status_msg;
      status_msg.stamp = last_state_reading.getStamp();
      status_msg.status_type.value = StatusType::GOAL_LOST;
      this->status_pub_.publish(status_msg);
      last_goal_reached_detected = false;
    }
  }

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  void NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::setWorldPerceptionPort(
      const shared_ptr<TWorldPerceptionPort>& perception_port)
  {
    this->perception_port_ = perception_port;
    this->registerComponent(this->perception_port_);
  }

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  void NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::setGoalPort(
      const shared_ptr<TGoalPort>& goal_port)
  {
    this->goal_port_ = goal_port;
    this->registerComponent(this->goal_port_);
  }

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  void NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::setStatePort(
      const shared_ptr<TStatePort>& state_correction_port)
  {
    this->state_correction_port_ = state_correction_port;
    this->registerComponent(this->state_correction_port_);
  }

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  void NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::setActionPort(
      const shared_ptr<TActionPort>& action_port)
  {
    this->action_port_ = action_port;
    this->registerComponent(this->action_port_);
  }
template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  void NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::setNavigationPlanner(
      const shared_ptr<TNavigationPlanner>& navigation_planner)
  {
    this->navigation_planner_ = navigation_planner;
    this->registerComponent(this->navigation_planner_);
  }
template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  void NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::setReachedGoalPredicate(
      const shared_ptr<TReachedGoalPredicate>& reach_goal_predicate)
  {
    this->reached_goal_predicate_ = reach_goal_predicate;
  }

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  void NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::setCollisionChecker(
      const shared_ptr<TCollisionChecker>& collision_detection)
  {
    this->collision_checker_ = collision_detection;
    this->registerComponent(this->collision_checker_);
  }

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  void NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::setStateEstimation(
      const shared_ptr<TStateEstimation>& state_estimation)
  {
    this->state_estimation_ = state_estimation;
    this->registerComponent(this->state_estimation_);

  }
template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  void NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::setKinodynamicModel(
      const shared_ptr<TKinoDynamicModel>& kinodynamic_model)
  {
    this->kinodynamic_model_ = kinodynamic_model;
    this->registerComponent(this->kinodynamic_model_);
  }
template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  void NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel>::setRobotShapeModel(
      const shared_ptr<TRobotShapeModel>& shape_model)
  {
    this->robot_shape_model_ = shape_model;
    this->registerComponent(this->robot_shape_model_);
  }

template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>

  template<class TInputCollisionChecker>
    void NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription,
        TimeModel>::setCovariantCollisionChecker(const shared_ptr<TInputCollisionChecker> collision_checker)
    {
      this->setCollisionChecker(
          rtcus_navigation::CovariantTraits<TCollisionChecker>::CovariantDecorator(collision_checker));
    }

}

#endif

