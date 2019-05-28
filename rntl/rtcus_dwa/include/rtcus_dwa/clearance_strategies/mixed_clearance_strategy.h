/*
 * mixed_clearance_strategy.h
 *
 *  Created on: Oct 9, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef MIXED_CLEARANCE_STRATEGY_H_
#define MIXED_CLEARANCE_STRATEGY_H_

#include <rtcus_dwa/clearance_strategies/separation_obstacle_cost_strategy.h>
namespace rtcus_dwa
{

class MixedClearanceCostStrategy : public ObstacleSeparationPonderatedClearanceCostStrategy
{
public:
  virtual ~MixedClearanceCostStrategy()
  {
  }

  MixedClearanceCostStrategy() :
      ObstacleSeparationPonderatedClearanceCostStrategy::ObstacleSeparationPonderatedClearanceCostStrategy()
  {

  }

  virtual t_float computeCost(double dtt_secs)
  {
    assert(
        this->config_->security_area_clearance_weight_obsolete >= 0.0 && this->config_->security_area_clearance_weight_obsolete <= 1.0);
    t_float default_cost = this->ObjectInfoDefaultClerance<ObstacleCollisionInfo>::computeCost(dtt_secs);

    t_float separance_cost = this->ObstacleSeparationPonderatedClearanceCostStrategy::computeCost(dtt_secs);

    return (1.0 - this->config_->security_area_clearance_weight_obsolete) * default_cost
        + (this->config_->security_area_clearance_weight_obsolete) * separance_cost;

  }

};
}

#endif /* MIXED_CLEARANCE_STRATEGY_H_ */
