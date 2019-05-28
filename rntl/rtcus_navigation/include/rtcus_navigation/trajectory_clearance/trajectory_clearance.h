/*
 * action_trajectory_clearance.h
 *
 *  Created on: Dec 10, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef ACTION_TRAJECTORY_CLEARANCE_H_
#define ACTION_TRAJECTORY_CLEARANCE_H_

#include <boost/shared_ptr.hpp>

namespace rtcus_navigation
{

namespace trajectory_clearance
{

#define TRAJECTORY_CLERANCE_TYPES(ActionType,ObstaclesType,RobotShapeType) \
  typedef ActionType TActionType; \
  typedef ObstaclesType TObstaclesType;\
  typedef RobotShapeType TRobotShapeType

/**
 * \brief This interface defines a method to decide the cost of a trajectory for a robot with an specific shape.
 * The result is typically the distance to the nearest obstacle. But it could also be another parameters like obstacle safety and others.
 * */
template<typename ActionType, typename ObstaclesType, typename RobotShapeType, typename CostType = double>
  class TrajectoryClearance
  {
  public:

    TRAJECTORY_CLERANCE_TYPES(ActionType,ObstaclesType,RobotShapeType);

    virtual bool computeClearance(const ActionType& action, const ObstaclesType& obstacles, const RobotShapeType& shape,
                                  CostType& clearance) const=0;

    virtual bool computeClearance(const ActionType& action, const ObstaclesType& obstacles, const RobotShapeType& shape,
                                  CostType& clearance)
    {
      return static_cast<const TrajectoryClearance*>(this)->computeClearance(action, obstacles, shape, clearance);
    }

    /*\brief max desired distance by the user. Can be useful both: for navigation customization and performance issues*/
    virtual void setMaxDistance(double distance)=0;
    virtual double getMaxDistance() const=0;

    virtual ~TrajectoryClearance()
    {
    }
  };
}
}

#endif /* TRAJECTORY_CLEARANCE_H_ */
