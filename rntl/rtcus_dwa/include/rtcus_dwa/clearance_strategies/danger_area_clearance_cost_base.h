/*
 * DangerAreaDetectionClearanceCostAspect.h
 *
 *  Created on: Oct 9, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef DANGERAREADETECTIONCLEARANCECOSTASPECT_H_
#define DANGERAREADETECTIONCLEARANCECOSTASPECT_H_
#include <rtcus_dwa/clearance_strategies/default_clearance_cost_strategy.h>
namespace rtcus_dwa
{

class DangerAreaClearanceCostBase : public ObjectInfoDefaultClerance<ObstacleCollisionInfo>
{
protected:
  bool isInsideDangerArea() const;
  bool isLoopingInsideDangerArea() const;
  virtual void preComputeCommand();

  virtual void onObstacleComputation(ObstacleCollisionInfo& obstacle_info);
  virtual void postObstacleInfoReduction(std::vector<ObstacleCollisionInfo>& obstacles);
  virtual void render() const;

public:
  virtual ~DangerAreaClearanceCostBase()
  {
  }

private:
  bool inside_danger_area_;
  bool danger_area_internal_loop_;

};
}

#endif /* DANGERAREADETECTIONCLEARANCECOSTASPECT_H_ */
