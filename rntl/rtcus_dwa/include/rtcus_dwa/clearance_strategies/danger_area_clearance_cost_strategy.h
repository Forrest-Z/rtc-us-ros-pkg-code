/*
 * default_clearance_cost_strategy.h
 *
 *  Created on: Oct 3, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef DANGER_AREA_CLEARANCE_COST_STRATEGY_H_
#define DANGER_AREA_CLEARANCE_COST_STRATEGY_H_

#include <rtcus_dwa/clearance_strategies/danger_area_clearance_cost_base.h>

namespace rtcus_dwa
{
class DangerAreaClearanceCostStrategy : public DangerAreaClearanceCostBase
{
protected:
  virtual void preComputeCommand() ;
  virtual double normalize();
  virtual void postObstacleInfoReduction(std::vector<ObstacleCollisionInfo>& obstacles);
  virtual t_float computeCost(double dtt_secs);

public:
  virtual void render() const;
  virtual ~DangerAreaClearanceCostStrategy();

private:
  double distance_entering_danger_area_;
  double distance_leaving_danger_area_;
  PointXY entering_point_;
  PointXY leaving_point_;

  //SECOND STAGE
  void setLeavingDangerAreaPoint(double dist, const PointXY& p);

  void setEnteringDangerAreaPoint(double dist, const PointXY& p);

  bool accesibleLeavingAttractivePoint() const;

  const PointXY& leavingAtractivePoint() const;
  double leavingDistance() const;

  bool accesibleEnteringRepulsivePoint() const;
  double enteringDistance() const;
  const PointXY& enteringRepulsivePoint() const;

};
}

#endif /* DEFAULT_CLEARANCE_COST_STRATEGY_H_ */
