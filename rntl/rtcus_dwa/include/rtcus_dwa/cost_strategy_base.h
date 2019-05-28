/*
 * cost_strategy_base.h
 *
 *  Created on: Feb 13, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef COST_STRATEGY_BASE_H_
#define COST_STRATEGY_BASE_H_
#include <limits>

namespace rtcus_dwa
{
template<typename ActionType>
  class CostStrategyBase
  {

  public:

    /**
     * \brief This is the main function of the class and defines how good is the given action
     * \param action the action to be evaluated
     * \returns the cost of the action action
     * */
    virtual t_float computeCost(const CommandCost<ActionType>& action) =0;

    virtual ~CostStrategyBase()
    {

    }
    virtual void init()
    {
      this->max_cost_ = numeric_limits<double>::min();
      this->min_cost_ = numeric_limits<double>::max();
    }

  private:
    t_float max_cost_;
    t_float min_cost_;

  protected:
    void normalizeDown(t_float cost)
    {

      if (cost < this->min_cost_)
        this->min_cost_ = cost;
    }

    void normalize(t_float cost)
    {
      if (this->max_cost_ < cost)
        this->max_cost_ = cost;
      if (cost < this->min_cost_)
        this->min_cost_ = cost;
    }

    void post_normalize(t_float inCost, t_float& outCost) const
    {
      //STREACH NORMAILZTION
      if (this->max_cost_ > numeric_limits<double>::min() && this->min_cost_ < numeric_limits<double>::max()
          && this->min_cost_ != this->max_cost_)
      {
        RTCUS_ASSERT_MSG(
            this->min_cost_ < this->max_cost_,
            "Bad use of the internal variables min and max of the specific cost strategy. (max %lf) (min %lf)",
            (double)this->max_cost_, (double)inCost);

        ROS_DEBUG(" * Post evaluating cost cost. Normalizing cost (%lf) to (min: %lf, max: %lf)", inCost,
                  this->min_cost_, this->max_cost_);
        double scale = max_cost_ - min_cost_;
        if (scale > 0.0)
          outCost = (inCost - min_cost_) / scale;
        else
          outCost = 0.0;

        if (outCost < 0)
          outCost = 0;

        ROS_DEBUG(" * Resulting cost: %lf", outCost);
      }
      //SIMPLE DOWN NORMALIZATION
      else if (this->max_cost_ > numeric_limits<double>::min())
      {
        outCost = inCost / this->max_cost_;
      }
      else if (this->min_cost_ == this->max_cost_)
      {
        outCost = this->min_cost_;
        ROS_WARN(" * Lose of entropy in cost normalization. max: %lf, min: %lf", this->min_cost_, this->max_cost_);
      }
      else
      {
        ROS_WARN(" * value of cost non normalized %lf. [min %lf, max %lf] Truncking cost value ", inCost,
                 this->min_cost_, this->max_cost_);
        outCost = std::min(1.0, std::max(inCost, 0.0));
        ROS_WARN(" * resulting: %lf ", outCost);
      }

      RTCUS_ASSERT_MSG(outCost >= 0.0 && outCost <= 1.0,
                       "ERROR ON COST COMPUTATION: (min %lf), (max: %lf)-> resulting (%lf)", (double)this->min_cost_,
                       (double)this->max_cost_, (double)outCost);

    }

  };
}

#endif /* COST_STRATEGY_BASE_H_ */
