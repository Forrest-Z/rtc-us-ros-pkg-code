/*
 * collision_checker_pointcloud2D_circular_robot.h
 *
 *  Created on: Dec 8, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_navigation/collision_checkers/collision_checker_pointcloud2D_circular_robot.h>

namespace rtcus_navigation
{
namespace collision_checkers
{
using namespace rtcus_robot_shapes;

CollisionChekerPointCloud2DForCircularRobot::CollisionChekerPointCloud2DForCircularRobot()
{
}

CollisionChekerPointCloud2DForCircularRobot::~CollisionChekerPointCloud2DForCircularRobot()
{
}
bool CollisionChekerPointCloud2DForCircularRobot::detectCollision(
    const pcl::PointCloud<pcl::PointXY> & obstacles,
    const rtcus_robot_shapes::interfaces::ICircularRobotShape& robot_shape) const
{
  bool collision = false;
  for (unsigned int i = 0; i < obstacles.points.size(); i++)
  {
    const pcl::PointXY& o = obstacles.points[i];
    collision = internal_point_collision_checker_.detectCollision(o, robot_shape);
    if (collision)
      break;
  }
  return collision;
}

void CollisionChekerPointCloud2DForCircularRobot::init()
{

}
void CollisionChekerPointCloud2DForCircularRobot::reset()
{

}
}
}

