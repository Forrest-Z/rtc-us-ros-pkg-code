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
namespace rtcus_navigation
{
namespace collision_checkers
{

using namespace rtcus_robot_shapes;
using namespace rtcus_robot_shapes::interfaces;
using namespace rtcus_navigation;

CollisionChekerTriangles::~CollisionChekerTriangles()
{
}
;

void CollisionChekerTriangles::init()
{

}

void CollisionChekerTriangles::reset()
{

}

inline bool CollisionChekerTriangles::insideTriangle(const pcl::PointXY& pt, const pcl::PointXY& v1,
                                                     const pcl::PointXY& v2, const pcl::PointXY& v3) const
{
  bool b1, b2, b3;

  b1 = Sign(pt, v1, v2) < 0.0f;
  b2 = Sign(pt, v2, v3) < 0.0f;
  b3 = Sign(pt, v3, v1) < 0.0f;

  return ((b1 == b2) && (b2 == b3));
}

inline float CollisionChekerTriangles::Sign(const pcl::PointXY& p1, const pcl::PointXY& p2,
                                            const pcl::PointXY& p3) const
{
  return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}

inline bool CollisionChekerTriangles::trianglesInclusionTest_aux(const pcl::PointXY& obstacle,
                                                                 const Triangles& triangles) const
{
  bool inside = false;
  for (unsigned int i = 0; !inside && i < triangles.size() - 2; i += 3)
  {
    //TODO: Remove this... this should be intrinsecally local
    //BROAD CHECKING
    if (obstacle.x <= this->circular_filter_center_.x + circular_filter_radio_
        && obstacle.y <= this->circular_filter_center_.y + circular_filter_radio_
        && obstacle.x >= this->circular_filter_center_.x - circular_filter_radio_
        && obstacle.y >= this->circular_filter_center_.y - circular_filter_radio_)
    {
      //Fine Checking
      const pcl::PointXY& p1 = triangles[i], &p2 = triangles[i + 1], &p3 = triangles[i + 2];
      //ROS_INFO_STREAM( "Cheking poing " << obstacle << "triangle: " << p1 << ", " << p2 << ", " << p3);
      inside = insideTriangle(obstacle, p1, p2, p3);
    }
  }
  return inside;
}

template<typename VectorLike>
  bool CollisionChekerTriangles::detectCollision_aux(const VectorLike& obstacles, const Triangles& triangles) const
  {
    bool collision = false;
    for (unsigned int i = 0; !collision && i < obstacles.size(); i++)
    {
      const PointXY& o = obstacles[i];
      collision = this->trianglesInclusionTest_aux(o, triangles);
    }
    return collision;
  }

//------------------------ COLLISION CHEKER INTERFACES --------------------------------------------------------------------------

bool CollisionChekerTriangles::detectCollision(const pcl::PointXY& obstacle, const Triangles& robot_shape) const
{
  return this->trianglesInclusionTest_aux(obstacle, robot_shape);
}

bool CollisionChekerTriangles::detectCollision(
    const std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> >& obstacles,
    const Triangles& robot_shape) const
{
  return this->detectCollision_aux(obstacles, robot_shape);
}

bool CollisionChekerTriangles::detectCollision(const pcl::PointCloud<pcl::PointXY>& obstacles,
                                               const Triangles& robot_shape) const
{
  return this->detectCollision_aux(obstacles.points, robot_shape);
}
bool CollisionChekerTriangles::detectCollision(const std::vector<pcl::PointXY>& obstacles,
                                               const Triangles& robot_shape) const
{
  return this->detectCollision_aux(obstacles, robot_shape);
}

//--------------------------------------------------------------------------------------------------

void CollisionChekerTriangles::setCircularRegionFilter(const PointXY& point, double radio)
{
  this->circular_filter_center_ = point;
  this->circular_filter_radio_ = radio;
}

}
}
