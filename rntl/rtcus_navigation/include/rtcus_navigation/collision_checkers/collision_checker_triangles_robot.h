/*
 * collision_checker_triangles_robot.h
 *
 *  Created on: Feb 25, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef COLLISION_CHECKER_TRIANGLES_ROBOT_H_
#define COLLISION_CHECKER_TRIANGLES_ROBOT_H_

#include <rtcus_navigation/common.h>
#include <rtcus_navigation/collision_checker.h>
#include <rtcus_robot_shapes/interfaces/triangles.h>
#include <Eigen/StdVector>

namespace rtcus_navigation
{
namespace collision_checkers
{

using namespace rtcus_robot_shapes;
using namespace rtcus_robot_shapes::interfaces;
using namespace rtcus_navigation;

class CollisionChekerTriangles : public CollisionChecker<pcl::PointXY, Triangles>, public CollisionChecker<
                                     pcl::PointCloud<pcl::PointXY>, Triangles>,
                                 public CollisionChecker<std::vector<pcl::PointXY>, Triangles>, public CollisionChecker<
                                     std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> >, Triangles>
{
public:
  virtual ~CollisionChekerTriangles();
  virtual void init();
  virtual void reset();
  //TODO: Remove this for global point checking and make it inherently local
  virtual void setCircularRegionFilter(const PointXY& point, double radious);
  bool use_shape_cache;

  virtual bool detectCollision(const pcl::PointCloud<pcl::PointXY>& obstacles, const Triangles& robot_shape) const;
  virtual bool detectCollision(const std::vector<pcl::PointXY>& obstacles, const Triangles& robot_shape) const;
  virtual bool detectCollision(const std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> >& obstacles,
                               const Triangles& robot_shape) const;
  virtual bool detectCollision(const pcl::PointXY& obstacle, const Triangles& robot_shape) const;

protected:
  double circular_filter_radio_;
  PointXY circular_filter_center_;

private:
  inline float Sign(const pcl::PointXY& p1, const pcl::PointXY& p2, const pcl::PointXY& p3) const;
  inline bool insideTriangle(const pcl::PointXY& pt, const pcl::PointXY& v1, const pcl::PointXY& v2,
                             const pcl::PointXY& v3) const;
  inline bool trianglesInclusionTest_aux(const pcl::PointXY& obstacle, const Triangles& triangles) const;
  template<typename VectorLike>
    inline bool detectCollision_aux(const VectorLike& obstacles, const Triangles& robot_shape) const;
};
}
}

#endif /* COLLISION_CHECKER_TRIANGLES_ROBOT_H_ */
