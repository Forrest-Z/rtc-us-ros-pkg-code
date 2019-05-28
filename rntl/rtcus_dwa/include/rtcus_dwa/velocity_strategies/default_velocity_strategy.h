/*
 * default_velocity_strategy.h
 *
 *  Created on: Dec 13, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef DEFAULT_VELOCITY_STRATEGY_H_
#define DEFAULT_VELOCITY_STRATEGY_H_
#include <rtcus_dwa/velocity_strategies/velocity_strategy.h>
namespace rtcus_dwa
{
namespace velocity_cost_strategies
{
/*\brief Tries to maximize the velocity, independent of the GoalType*/
template<typename GoalType>
  class DefaultVelocityStrategy : public VelocityStrategy<Twist2D, GoalType>
  {

  public:
    virtual ~DefaultVelocityStrategy()
    {

    }

    virtual double computeCost(const CommandCost<Twist2D>& action)
    {
      double velocity_cost;
      if (this->config_->get_v_botom() == this->config_->get_v_top())
      {
        ROS_WARN("Dynamic Window without area. Velocity Cost couldn't be computed properly.");
        velocity_cost = 0.0;
      }
      else
      {
        velocity_cost = 1.0
            - (fabs(action.getAction().linear - this->config_->get_v_botom()))
                / fabs(this->config_->get_v_botom() - this->config_->get_v_top());
        RTCUS_ASSERT_MSG(velocity_cost >= 0.0 && velocity_cost <= 1.0, "velocity cost -> %lf", velocity_cost);

      }
      return velocity_cost;
    }
    virtual void postEvaluation(CommandCost<Twist2D>& cmd)
    {
      //DO NONE. ALREADY NORMALIZED
    }
  };

}
}

#endif /* DEFAULT_VELOCITY_STRATEGY_H_ */
