/*
 * simple_dwa.h
 *
 *  Created on: 29/11/2011
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef SIMPLE_DWA_H_
#define SIMPLE_DWA_H_
#include <rtcus_dwa/common.h>
#include <rtcus_dwa/dwa_config.h>
#include <rtcus_dwa/dwa_command_cost.h>
#include <rtcus_dwa/action_space.h>
#include <rtcus_dwa/dwa_motion_model.h>

#include <rtcus_dwa/clearance_strategies/default_clearance_cost_strategy.h>
#include <rtcus_dwa/clearance_strategies/danger_area_clearance_cost_strategy.h>
#include <rtcus_dwa/clearance_strategies/separation_obstacle_cost_strategy.h>
#include <rtcus_dwa/clearance_strategies/mixed_clearance_strategy.h>

#include <rtcus_dwa/velocity_strategies/velocity_strategy.h>
#include <rtcus_dwa/heading_cost_strategies.h>

#include <vector>
#include <boost/signals.hpp>

namespace rtcus_dwa
{
using namespace boost;

/**
 * \brief DWA Algorithm without fitting the rtcus_navigation::NavigationPlanner interface (this will be extended to do it)
 * */
template<typename GoalType, typename ActionType>
  class Dwa2DAlgorithm
  {
  public:
    Dwa2DAlgorithm();
    virtual ~Dwa2DAlgorithm();
    void init(ros::NodeHandle& host_node_);
    void reset();
    virtual bool computeVelocityCommands(ActionType &out);
    virtual bool evaluateCommand(CommandCost<ActionType>& action, const Twist2D& current_state);

    boost::signal<void(const CommandCost<ActionType>& action)> onActionEvaluated;

  protected:
    ros::Publisher dwa_status_publisher_;
    t_float last_duration_period_;

    /*Local state description. The global pose does not matter since it is solved outside*/
    Twist2D last_state_;
    GoalType goal_;
    const pcl::PointCloud<PointXY>* obstacles_;

    void evaluateTotalCost(const t_float min_heading, const t_float max_heading,
                           const t_float total_command_max_obstacle_distance, ActionType &best_command);

  public:
    shared_ptr<ActionSpace<ActionType, DwaConfig> > action_space_;
    shared_ptr<HeadingCostStrategy<GoalType, /*StateType*/Twist2D, ActionType> > heading_cost_strategy_;
    shared_ptr<VelocityStrategy<ActionType, GoalType> > velocity_strategy_;
    shared_ptr<rtcus_dwa::ClearanceCostStrategyBase> command_clearance_;
    shared_ptr<CollisionCheker> collision_checker_;

    rtcus_dwa::DWAMotionModel motion_model_;
    const rtcus_dwa::DWAMotionModel& getMotionModel() const;
    DwaConfig config_;

    //----------------- AUXILIAR METHODS -------------------------------------
    void setState(Twist2D current_state_estimation, t_float time_period);
    virtual void setObstacles(const pcl::PointCloud<PointXY>& obstacle_data);
    bool obstacles_initializated() const;
    virtual void setGoal(const GoalType& goal)
    {
      this->goal_ = goal;
    }
    virtual inline bool preconditions()
    {
      return obstacles_ && last_state_.linear == last_state_.linear && last_state_.angular == last_state_.angular
          && last_state_.lateral == last_state_.lateral && last_duration_period_ == last_duration_period_;
    }
    void updateConfig(const SimpleDwaConfig& config);
    const DwaConfig& getConfig() const;
    void publish_dwa_status(float millisecs_duration, const CommandCost<Twist2D>& best_command, const GoalType& goal);
  };

}
;

#endif
