/*
 * no_collision_checking.h
 *
 *  Created on: Nov 23, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef NO_COLLISION_CHECKING_H_
#define NO_COLLISION_CHECKING_H_
#include <rtcus_navigation/collision_checker.h>
namespace rtcus_navigation
{

namespace collision_checkers
{

/**
 * \brief this strategy is useful if no collision checking is required but something has to be specified for the NavigationNode.
 * */
template<typename ObstaclesType, typename RobotShape>
  class NoCollisionChecking : public CollisionChecker<ObstaclesType, RobotShape>
  {
  public:

    virtual ~NoCollisionChecking()
    {
    }
    ;

    virtual bool detectCollision(const ObstaclesType& obstacles, const RobotShape& robot_shape) const
    {
      return false;
    }
    virtual void init()
    {
    }
    virtual void reset()
    {
    }
  };
}
}

#endif /* NO_COLLISION_CHECKING_H_ */
