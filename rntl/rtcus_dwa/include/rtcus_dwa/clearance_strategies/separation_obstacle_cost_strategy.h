/*
 * separation_obstacle_cost_strategy.h
 *
 *  Created on: Oct 3, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef SEPARATION_OBSTACLE_COST_STRATEGY_H_
#define SEPARATION_OBSTACLE_COST_STRATEGY_H_

#include <rtcus_dwa/clearance_strategies/default_clearance_cost_strategy.h>

namespace rtcus_dwa
{
class ObstacleSeparationClearanceCostStrategy : public ObjectInfoDefaultClerance<ObstacleCollisionInfo>
{
public:
  virtual ~ObstacleSeparationClearanceCostStrategy();
  virtual void preComputeCommand();

  bool hasRepulsivePoint() const;
  const ObstacleCollisionInfo& getRepulsivePoint() const;
  virtual void render() const;

protected:
  ObstacleCollisionInfo repulsive_point_;
  bool has_repulsive_point_;

  virtual void onObstacleComputation(ObstacleCollisionInfo& obstacle_info);
  virtual void setRepulsivePoint(const ObstacleCollisionInfo& proyected_collision_);
  virtual void postObstacleInfoReduction(std::vector<ObstacleCollisionInfo>& obstacles);
  virtual t_float computeCost(double dtt_secs);
  t_float getNormalRepulsionObstacleCost(float separation_distance, float linear_distance) const;
}
;


class ObstacleSeparationPonderatedClearanceCostStrategy : public ObstacleSeparationClearanceCostStrategy
{
public:
  virtual ~ObstacleSeparationPonderatedClearanceCostStrategy()
  {
  }

protected:
  t_float getNormalRepulsionObstacleCost(float separation_distance, float linear_distance) const;

}
;

}

#endif /* SEPARATION_OBSTACLE_COST_STRATEGY_H_ */
