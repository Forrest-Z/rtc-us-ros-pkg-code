/*
 * collision_cheker_polygonal_robot.h
 *
 *  Created on: Feb 20, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */
#include <rtcus_navigation/collision_checkers/collision_cheker_polygonal_robot.h>
#include <rtcus_robot_shapes/polygonal_robot.h>
#include <rtcus_robot_shapes/shape_operations.h>
#include <Eigen/StdVector>

namespace rtcus_navigation
{
namespace collision_checkers
{

using namespace rtcus_robot_shapes;
using namespace rtcus_robot_shapes::interfaces;
using namespace rtcus_navigation;

CollisionChekerPolygonalRobot::~CollisionChekerPolygonalRobot()
{
}
;

bool CollisionChekerPolygonalRobot::detectCollision(const pcl::PointXY& obstacle,
                                                    const IPolygonalRobotShape& robot_shape) const
{
  cacheTriangularBufferFromPolygon(robot_shape);
  return CollisionChekerTriangles::detectCollision(obstacle, this->triangle_buffer_);
}

bool CollisionChekerPolygonalRobot::detectCollision(const std::vector<pcl::PointXY>& obstacles,
                                                    const IPolygonalRobotShape& robot_shape) const
{
  cacheTriangularBufferFromPolygon(robot_shape);
  return CollisionChekerTriangles::detectCollision(obstacles, this->triangle_buffer_);
}

bool CollisionChekerPolygonalRobot::detectCollision(const pcl::PointCloud<pcl::PointXY>& obstacles,
                                                    const IPolygonalRobotShape& robot_shape) const
{
  cacheTriangularBufferFromPolygon(robot_shape);
  return CollisionChekerTriangles::detectCollision(obstacles, this->triangle_buffer_);
}

void CollisionChekerPolygonalRobot::cacheTriangularBufferFromPolygon(const IPolygonalRobotShape& robot_shape) const
{
  //if (use_shape_cache && this->triangle_buffer_.size() > 0)
  //  return;
  //caching this triangle violates the const attribute
  Triangles& triangle_buffer = const_cast<CollisionChekerPolygonalRobot*>(this)->triangle_buffer_;
  double radio;
  rtcus_robot_shapes::getTriangleBufferFromPolygon(robot_shape, triangle_buffer, radio);
}

}
}
