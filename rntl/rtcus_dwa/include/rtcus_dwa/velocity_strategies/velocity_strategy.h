/*
 * velocity_strategy.h
 *
 *  Created on: Feb 13, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef VELOCITY_STRATEGY_H_
#define VELOCITY_STRATEGY_H_

#include <rtcus_dwa/dwa_config.h>
#include <rtcus_dwa/cost_strategy_base.h>

namespace rtcus_dwa
{
/*\brief this interface defines a method to evaluate the cost of the linear velocity given an specific action command*/
//TODO: move this code to a linkable unit
template<typename ActionType, typename GoalType>
  class VelocityStrategy : public CostStrategyBase<ActionType>
  {
  protected:
    const DwaConfig* config_;
    const GoalType* goal_;

  public:
    virtual ~VelocityStrategy()
    {
    }

    virtual void init(const rtcus_dwa::DwaConfig& config, const GoalType& goal)
    {
      CostStrategyBase<ActionType>::init();
      this->config_ = &config;
      this->goal_ = &goal;
    }

    virtual void postEvaluation(CommandCost<ActionType>& cmd)
    {
      double velocity;
      this->post_normalize(cmd.getVelocity(), velocity);
      cmd.setVelocity(velocity);
    }
  };
}

#endif /* VELOCITY_STRATEGY_H_ */
