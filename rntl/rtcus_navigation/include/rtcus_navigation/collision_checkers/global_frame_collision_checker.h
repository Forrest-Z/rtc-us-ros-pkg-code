/*
 * global_frame_collision_checker.h
 *
 *  Created on: Dec 15, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef GLOBAL_FRAME_COLLISION_CHECKER_H_
#define GLOBAL_FRAME_COLLISION_CHECKER_H_

#include <rtcus_navigation/collision_checker.h>

namespace rtcus_navigation
{
namespace collision_checkers
{
/**
 *  \brief Generalizes the behavior of the CollisionChecker interface for working with a global description of the obstacles given
 *  the current state or pose of the robot.
 *
 *  TODO: Try to do a sfinae with a traits class to check in compile time if the StateType is compatible with the
 *  current Navigation Node.
 * */
template<typename ObstaclesType, typename RobotShapeType, typename StateType>
  class GlobalFrameCollisionChecker : public CollisionChecker<ObstaclesType, RobotShapeType>
  {

  protected:
    shared_ptr<CollisionChecker<ObstaclesType, RobotShapeType> > decorated_;
  public:
    GlobalFrameCollisionChecker(shared_ptr<CollisionChecker<ObstaclesType, RobotShapeType> > decorated)
    {

    }
    virtual ~GlobalFrameCollisionChecker()
    {
    }
    virtual bool detectCollision(const ObstaclesType& obstacles, const RobotShapeType& robot_shape,
                                 const StateType& current_state) const
    {
      /*
       rtcus_compositions::StateComposer::compose(original_triangles, trajectory[i], triangles);
       PointXY center;
       center.x = trajectory[i].x;
       center.y = trajectory[i].y;
       this->collision_checker_->setCircularRegionFilter(center, radio);
       collision = this->collision_checker_->detectCollision(obstacles, triangles);
       */
      return false;
    }
  };
}
}

#endif /* GLOBAL_FRAME_COLLISION_CHECKER_H_ */
