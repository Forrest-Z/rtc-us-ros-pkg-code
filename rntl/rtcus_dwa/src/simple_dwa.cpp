/*
 * simple_dwa.cpp
 *
 *  Created on: 22/12/2011
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_dwa/simple_dwa.h>
#include <rtcus_dwa/dwa_config.h>
#include <ros/ros.h>
#include <boost/make_shared.hpp>
#include <rtcus_assert/rtcus_assert.h>
#include <rtcus_dwa/DwaStatus.h>
#include <limits>

using namespace std;

namespace rtcus_dwa
{

template<typename GoalType, typename ActionType>
  Dwa2DAlgorithm<GoalType, ActionType>::Dwa2DAlgorithm() :
      command_clearance_()
  {
    this->collision_checker_ = boost::make_shared<rtcus_dwa::CollisionCheker>();
    this->action_space_ = boost::make_shared<rtcus_dwa::DwaActionSpace>();
    this->command_clearance_ = boost::make_shared<rtcus_dwa::ClearanceCostStrategyBase>();
    this->command_clearance_->setCollisionChecker(this->collision_checker_);
    reset();
  }

template<typename GoalType, typename ActionType>
  Dwa2DAlgorithm<GoalType, ActionType>::~Dwa2DAlgorithm()
  {

  }

template<typename GoalType, typename ActionType>
  void Dwa2DAlgorithm<GoalType, ActionType>::init(ros::NodeHandle& host_node_)

  {
    this->dwa_status_publisher_ = host_node_.advertise<DwaStatus>("dwa_status", 100, false);
  }

//-------------------------------------

template<typename GoalType, typename ActionType>
  const rtcus_dwa::DWAMotionModel& Dwa2DAlgorithm<GoalType, ActionType>::getMotionModel() const
  {
    return motion_model_;
  }

template<typename GoalType, typename ActionType>
  const DwaConfig& Dwa2DAlgorithm<GoalType, ActionType>::getConfig() const
  {
    return this->config_;
  }

template<typename GoalType, typename ActionType>
  void Dwa2DAlgorithm<GoalType, ActionType>::updateConfig(const SimpleDwaConfig& config)
  {
    this->config_.updateConfig(config);
    ROS_INFO("Calling to clearance lookup table rebuilding");
    this->command_clearance_->onRobotShapeChange(this->config_.getRobotShape(), this->config_);
  }

template<typename GoalType, typename ActionType>
  void Dwa2DAlgorithm<GoalType, ActionType>::reset()

  {
    obstacles_ = NULL;
    last_state_.linear = numeric_limits<t_float>::quiet_NaN();
    last_state_.angular = numeric_limits<t_float>::quiet_NaN();
    last_state_.lateral = numeric_limits<t_float>::quiet_NaN();
    last_duration_period_ = numeric_limits<t_float>::quiet_NaN();

    //for instance to reload parameters
    this->action_space_->reset();
  }
template<typename GoalType, typename ActionType>
  bool Dwa2DAlgorithm<GoalType, ActionType>::obstacles_initializated() const
  {
    return obstacles_ && obstacles_->size() > 0;
  }

template<typename GoalType, typename ActionType>
  void Dwa2DAlgorithm<GoalType, ActionType>::setState(Twist2D current_state_estimation, t_float duration_period)
  {
    last_state_ = current_state_estimation;
    last_state_.lateral = 0;
    last_duration_period_ = duration_period;
  }

//shared pointer data, no thread safe
template<typename GoalType, typename ActionType>
  void Dwa2DAlgorithm<GoalType, ActionType>::setObstacles(const pcl::PointCloud<PointXY>& obstacle_data)
  {
    this->obstacles_ = &obstacle_data;
    //check coherence and look for nans
    for (unsigned int i = 0; i < obstacles_->points.size(); i++)
      RTCUS_ASSERT(
          obstacles_->points[i].x == obstacles_->points[i].x && obstacles_->points[i].y == obstacles_->points[i].y);
  }

/*
 //this method is called periodically to actualize the goal position in the local framepp
 template<typename GoalType, typename ActionType>
 void Dwa2DAlgorithm<GoalType, ActionType>::setGoal(const GoalType& goal)
 {
 this->goal_ = goal;
 }
 */

template<typename GoalType, typename ActionType>
  bool Dwa2DAlgorithm<GoalType, ActionType>::evaluateCommand(CommandCost<ActionType>& action, const Twist2D& state)
  {

    //-------------------- HEADING COST ---------------------------
    action.setHeading(heading_cost_strategy_->computeCost(action));

    //-------------------- VELOCITY COST ---------------------------
    action.setVelocity(this->velocity_strategy_->computeCost(action));

    //-------------------- CLEARANCE COST ---------------------------
    this->command_clearance_->computeClearance(action, *this->obstacles_, this->config_.getRobotShape(), action);

    //-------------------- ADMISIBILITY COMPUTATION ---------------------------
    action.computeAdmisibility(this->config_.getKinodynamicConfig());

    this->onActionEvaluated(action);

    return true;
  }

template<typename GoalType, typename ActionType>
  bool Dwa2DAlgorithm<GoalType, ActionType>::computeVelocityCommands(ActionType &out)
  {
    RTCUS_ASSERT(preconditions());

    if (!this->obstacles_initializated())
      return false;

    t_float& time_duration_period = last_duration_period_;
    bool ok = config_.update_window(last_state_.linear, last_state_.angular, time_duration_period);
    if (!ok)
      RTCUS_ASSERT_MSG(
          false,
          "DWA core. Bad behaving of the DWA Algorithm. Check the update algorithm of the DWAConfig. Bad behaving of the DWA Algorithm");

    RTCUS_ASSERT(obstacles_initializated());

    action_space_->onConfigUpdate(config_);
    this->config_.setCollisionState(
        this->collision_checker_->detectCollision(*this->obstacles_, this->config_.getRobotShape()));

    //CHECK COLLISION
    if (this->config_.hasCollision())
    {
      //dinamically posible emergency stop
      Twist2D stop_action;
      if (this->last_state_.angular > 0)
        stop_action.angular = max(0.0, config_.get_omega_left());
      else
        stop_action.angular = min(0.0, config_.get_omega_right());

      if (this->last_state_.linear > 0)
        stop_action.linear = max(0.0, config_.get_v_botom());
      else
        stop_action.linear = min(0.0, config_.get_v_top());

      out = stop_action;
      return false;
    }
    else //DO THE PLAN NORMALY
    {
      //----------------------------------------------------------------------------
      //STEP 0 - global initialization of the cost strategies
      this->heading_cost_strategy_->init(config_, this->last_state_, this->motion_model_, this->goal_);
      this->velocity_strategy_->init(config_, this->goal_);
      this->command_clearance_->init(config_);
      this->action_space_->initialize(config_);

      //----------------------------------------------------------------------------------
      //STEP 1 - CALCULATE RAW METRICS: distance, velocity and heading
      for (unsigned int action_index = 0; action_index < action_space_->size(); action_index++)
      {
        CommandCost<ActionType> &action = action_space_->getAction(action_index);
        evaluateCommand(action, last_state_);
        if (this->config_.hasCollision())
          action.setNonAdmisibleCommand(1.0);
      }

      //---------------------------------------------------------------------------
      //STEP 2 - POSTEVALUATION: Normalizations, etc...
      for (unsigned int action_index = 0; action_index < action_space_->size(); action_index++)
      {
        CommandCost<ActionType> &command_cost = action_space_->getAction(action_index);
        this->heading_cost_strategy_->postEvaluation(command_cost);
        this->velocity_strategy_->postEvaluation(command_cost);
      }

      //----------------------------------------------------------------------------------
      //STEP 3- GLOBAL AGGREGATION
      action_space_->globalEvaluate();

      //----------------------------------------------------------------------------------
      //STEP 4 - CHECK COHERENCE & SELECT BEST COMMAND
      const CommandCost<ActionType>* best_command = NULL;

      for (unsigned int action_index = 0; action_index < action_space_->size(); action_index++)
      {
        const CommandCost<ActionType> &action = action_space_->getAction(action_index);
        RTCUS_ASSERT_MSG(action.getClearance() >= 0.0 && action.getClearance() <= 1.0,
                         " clearance: %lf", (double)action.getClearance());
        RTCUS_ASSERT_MSG(action.getHeading() >= 0.0 && action.getHeading() <= 1.0,
                         " heading: %lf", (double)action.getHeading());
        RTCUS_ASSERT_MSG(action.getVelocity() >= 0.0 && action.getVelocity() <= 1.0,
                         " velocity: %lf", (double)action.getVelocity());
        RTCUS_ASSERT_MSG(action.getTotalCost() >= 0.0 && action.getTotalCost() <= 1.0,
                         " total: %lf", (double)action.getTotalCost());

        //IGNORE NON ADMISIBLE COMMANDS
        if (!action.isAdmisibleCommand())
        {
          ROS_DEBUG(
              " v %lf omega %lf NonAdmisible || inside obstacle", action.getAction().linear, action.getAction().angular);
          continue;
        }
        else
        {
          if (best_command == NULL || action.getTotalCost() < best_command->getTotalCost())
            best_command = &action;
        }
      }

//-----------------------------------------------------------------------------------------
// STEP 5 - FINISH AND RECOVERY BEHAVIOURS
      if (action_space_->admisibilitySaturated())
      {
        //TODO: THIS SHOULD BE A RECOVERY BEHAVIOUR
        double best_total = numeric_limits<double>::max();
        t_float omega = config_.get_omega_left();
        int offset = config_.get_resolution_width() / 2;
        for (unsigned int j = 0; j < config_.get_resolution_width(); omega += config_.omega_step, j++)
        {
          CommandCost<ActionType>& command = action_space_->getAction((j + offset) % config_.get_resolution_width());
          //TODO: minimize the speed collision
          double total = command.getClearance();
          if (total < best_total)
          {
            if (best_command != NULL)
              ROS_WARN(
                  "Selecting emergency command due to admissibility saturation %lf %lf", best_command->getAction().linear, best_command->getAction().angular);

            best_command = &command;
            best_total = total;
          }
        }
        ROS_WARN(
            "Admissibility Saturation. Current linear %lf angular %lf, selected %lf %lf", last_state_.linear, last_state_.angular, best_command->getAction().linear, best_command->getAction().angular);
      }
      if (best_command != NULL)
      {
        out = best_command->getAction();
        this->publish_dwa_status(0.0, *best_command, this->goal_);
        return true;
      }
      else
        return false;
    }
    return true;
  }

//TODO: Obsolete, to remove or move to the specific DWA algorithm
template<typename GoalType, typename ActionType>
  void Dwa2DAlgorithm<GoalType, ActionType>::publish_dwa_status(float millisecs_duration,
                                                                const CommandCost<Twist2D>& best_command,
                                                                const GoalType& goal)
  {
    const DwaConfig& current_config = this->getConfig();
    //Publishing Status Information for plotting
    DwaStatus status_msg;
    status_msg.dwa_loop_time = millisecs_duration;

    if (!best_command.isAdmisibleCommand())
      status_msg.heading_cost = status_msg.velocity_cost = status_msg.clearance_cost = 1.0;
    else if (this->obstacles_initializated())
      status_msg.heading_cost = status_msg.velocity_cost = status_msg.clearance_cost = 0;
    else
    {
      rtcus_nav_msgs::Twist2D command;
      command.linear = best_command.getAction().linear;
      command.angular = best_command.getAction().angular;
      command.lateral = 0;
      DynamicState2D predicted_state;

      this->motion_model_.predictLocalStateTransition(command, ros::Duration(current_config.simulation_time_step),
                                                      predicted_state);

      status_msg.heading_cost = best_command.getHeading();
//status_msg.heading_error = goal_angle - predicted_phi;
      status_msg.velocity_cost = best_command.getVelocity();
      status_msg.clearance_cost = best_command.getClearance();
    }

    status_msg.header.stamp = ros::Time::now();
    dwa_status_publisher_.publish(status_msg);
  }

template class Dwa2DAlgorithm<rtcus_dwa::PointXY, rtcus_dwa::Twist2D> ;
template class Dwa2DAlgorithm<rtcus_dwa::Twist2D, rtcus_dwa::Twist2D> ;
}
;
