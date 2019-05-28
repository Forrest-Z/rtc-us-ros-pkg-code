/*
 * collision_checker_pointxy_circular_robot.h
 *
 *  Created on: Dec 8, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef COLLISION_CHECKER_POINTXY_CIRCULAR_ROBOT_H_
#define COLLISION_CHECKER_POINTXY_CIRCULAR_ROBOT_H_

#include <rtcus_navigation/common.h>
#include <rtcus_navigation/collision_checker.h>
#include <rtcus_robot_shapes/interfaces/circular.h>

namespace rtcus_navigation
{
namespace collision_checkers
{
/**
 * \brief This class detects the collision between a circular robot and a pointXY obstacle. This simple classes are useful to be used in combination
 * with lookup-table methods.
 *
 * TODO: This collision checker could be much more independent defining a generic interface about a circular robot.
 * For instance, a polygonal robot could be used based on its circunscribed radius.
 * */
using namespace rtcus_robot_shapes;
class CollisionChekerPoint2DForCircularRobot : public rtcus_navigation::CollisionChecker<pcl::PointXY,
    rtcus_robot_shapes::interfaces::ICircularRobotShape>
{
public:

  typedef rtcus_navigation::CollisionChecker<pcl::PointXY, rtcus_robot_shapes::interfaces::ICircularRobotShape> TCollisionChecker;

  virtual ~CollisionChekerPoint2DForCircularRobot();
  virtual void init();
  virtual void reset();
  virtual bool detectCollision(const pcl::PointXY& obstacle,
                               const rtcus_robot_shapes::interfaces::ICircularRobotShape& robot_shape) const;

};
}
}

#endif /* COLLISION_CHECKER_POINTXY_CIRCULAR_ROBOT_H_ */
