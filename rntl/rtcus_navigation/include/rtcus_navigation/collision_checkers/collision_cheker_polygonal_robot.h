/*
 * collision_cheker_polygonal_robot.h
 *
 *  Created on: Feb 20, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef COLLISION_CHEKER_POLYGONAL_ROBOT_H_
#define COLLISION_CHEKER_POLYGONAL_ROBOT_H_

#include <rtcus_navigation/common.h>
#include <rtcus_navigation/collision_checker.h>
#include <rtcus_robot_shapes/interfaces/polygonal.h>
#include <rtcus_navigation/collision_checkers/collision_checker_triangles_robot.h>

namespace rtcus_navigation
{
namespace collision_checkers
{

using namespace rtcus_robot_shapes;
using namespace rtcus_robot_shapes::interfaces;
using namespace rtcus_navigation;

/**
 * \brief This class check if a polygonal robot collides with one or many pointXY obstacles
 * */
//TODO: To improve this.. split robot shapes of <Triangles> and Polygon extending Triangles
class CollisionChekerPolygonalRobot : public rtcus_navigation::collision_checkers::CollisionChekerTriangles,
                                      public CollisionChecker<pcl::PointXY, IPolygonalRobotShape>,
                                      public CollisionChecker<std::vector<pcl::PointXY>, IPolygonalRobotShape>,
                                      public CollisionChecker<pcl::PointCloud<pcl::PointXY>, IPolygonalRobotShape>

{
public:
  //TODO; Remove this approach
  //main covariant type
  typedef CollisionChecker<pcl::PointCloud<pcl::PointXY>, IPolygonalRobotShape> TCollisionChecker;

protected:
  Triangles triangle_buffer_;
  //this method is though for those shapes which do not cache is own triangularization
  void cacheTriangularBufferFromPolygon(const IPolygonalRobotShape& robot_shape) const;

public:
  virtual ~CollisionChekerPolygonalRobot();

  virtual void init()
  {
  }
  virtual void reset()
  {
  }

  virtual bool detectCollision(const pcl::PointXY& obstacle, const IPolygonalRobotShape& robot_shape) const;

  virtual bool detectCollision(const std::vector<pcl::PointXY>& obstacles,
                               const IPolygonalRobotShape& robot_shape) const;

  virtual bool detectCollision(const pcl::PointCloud<pcl::PointXY>& obstacles,
                               const IPolygonalRobotShape& robot_shape) const;

};
}
}

#endif /* COLLISION_CHEKER_POLYGONAL_ROBOT_H_ */
