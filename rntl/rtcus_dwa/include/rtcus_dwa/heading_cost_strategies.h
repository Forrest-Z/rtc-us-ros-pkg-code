/*
 * heading_cost_strategies.h
 *
 *  Created on: Aug 28, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef HEADING_COST_STRATEGIES_H_
#define HEADING_COST_STRATEGIES_H_

#include <rtcus_dwa/dwa_motion_model.h>
#include <rtcus_dwa/dwa_config.h>
#include <rtcus_dwa/dwa_command_cost.h>
#include <rtcus_dwa/cost_strategy_base.h>

namespace rtcus_dwa
{

using namespace std;

/* *
 * \brief Defines a criteria about how well the is robot getting closer to the target.  This class defines an interface to be inherited.
 * This class has been designed as an interface to be inherited. In general,
 * */
template<typename GoalType, typename StateType, typename ActionType>
  class HeadingCostStrategy : public CostStrategyBase<ActionType>
  {

  public:
    virtual ~HeadingCostStrategy()
    {
    }
    /**
     * \brief This method should be called once in each navigation step and before the individual analysis of each individual command action.
     * */
    virtual void init(const DwaConfig& config, const StateType& state, const DWAMotionModel& motion_model,
                      const GoalType& goal)
    {
      CostStrategyBase<ActionType>::init();
      this->config_ = &config;
      this->goal_ = &goal;
      this->motion_model_ = &motion_model;
      this->state_ = &state;
    }

    /**
     * \brief Typically used for normalization. The default normalization uses the attribute max_heading_ if has been used.
     * */
    virtual void postEvaluation(CommandCost<ActionType>& cmd) const
    {
      t_float post_procesed;
      this->post_normalize(cmd.getHeading(), post_procesed);
      cmd.setHeading(post_procesed);
    }

  protected:
    const DwaConfig* config_;
    const DWAMotionModel* motion_model_;
    const GoalType* goal_;
    const StateType* state_;
  };

;
}

;
#endif /* HEADING_COST_STRATEGIES_H_ */
