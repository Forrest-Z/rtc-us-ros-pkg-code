/*
 * collision_detection.cpp
 *
 *  Created on: Oct 7, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_navigation/collision_checkers/collision_checker_pointxy_circular_robot.h>
#include <rtcus_nav_msgs/DynamicState2D.h>
#include <rtcus_assert/rtcus_assert.h>

namespace rtcus_navigation
{
namespace collision_checkers
{
using namespace rtcus_robot_shapes;


CollisionChekerPoint2DForCircularRobot::~CollisionChekerPoint2DForCircularRobot()
{
}

bool CollisionChekerPoint2DForCircularRobot::detectCollision(
    const pcl::PointXY& obstacle, const rtcus_robot_shapes::interfaces::ICircularRobotShape& robot_shape) const
{
  return obstacle.x * obstacle.x + obstacle.y * obstacle.y < robot_shape.getRadius()*robot_shape.getRadius();
}

void CollisionChekerPoint2DForCircularRobot::init()
{

}
void CollisionChekerPoint2DForCircularRobot::reset()
{

}
}
}

