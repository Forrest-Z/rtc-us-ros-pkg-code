/*
 *
 *  Created on: Feb 19, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_navigation/trajectory_clearance/discrete_collision_shaped_robot_clearance.h>
#include <vector>
#include <rtcus_assert/rtcus_assert.h>
#include <rtcus_compositions/state_composer.h>
#include <math.h>
#include <rtcus_navigation/collision_checkers/collision_cheker_polygonal_robot.h>
#include <rtcus_robot_shapes/shape_operations.h>

namespace rtcus_navigation
{
namespace trajectory_clearance
{
using namespace rtcus_robot_shapes::interfaces;
using namespace std;
using namespace boost;

DiscreteCollisionShapedRobotClearance::DiscreteCollisionShapedRobotClearance() :
    max_distance_(std::numeric_limits<double>::max()), strategy(0)
{
  this->collision_checker_ = make_shared<rtcus_navigation::collision_checkers::CollisionChekerTriangles>();
}

DiscreteCollisionShapedRobotClearance::DiscreteCollisionShapedRobotClearance(
    shared_ptr<rtcus_navigation::collision_checkers::CollisionChekerTriangles> collision_checker) :
    max_distance_(std::numeric_limits<double>::max()), strategy(0)
{
  this->collision_checker_ = collision_checker;
}
DiscreteCollisionShapedRobotClearance::~DiscreteCollisionShapedRobotClearance()
{

}

void DiscreteCollisionShapedRobotClearance::setMaxDistance(double distance)
{
  max_distance_ = distance;
}

double DiscreteCollisionShapedRobotClearance::getMaxDistance() const
{
  return max_distance_;
}

template<typename VectorLike>
  inline void filter_radius(const VectorLike& obstacles, VectorLike& result, const Pose2D& pose, double radio)
  {
    result.clear();
    int cont = 0;
    for (unsigned int i = 0; i < obstacles.size(); i++)
    {
      const PointXY& o = obstacles[i];
      if (o.x <= pose.x + radio && o.y <= pose.y + radio && o.x >= pose.x - radio && o.y >= pose.y - radio)
      {
        result.push_back(o);
        cont++;
      }
    }
    result.resize(cont);
  }

template<typename VectorLike>
  inline void filter_radius_global(const VectorLike& obstacles, VectorLike& result, const Pose2D& pose,
                                   const PointXY& center, double radio)
  {
    result.clear();
    int cont = 0;
    for (unsigned int i = 0; i < obstacles.size(); i++)
    {
      const PointXY& o = obstacles[i];
      if (o.x <= pose.x + radio && o.y <= pose.y + radio && o.x >= pose.x - radio && o.y >= pose.y - radio)
      {
        result.push_back(o);
        cont++;
      }
    }
    result.resize(cont);
  }

template<typename VectorLike>
  inline bool DiscreteCollisionShapedRobotClearance::computeClerance_auxb(const vector<Pose2D>& trajectory,
                                                                          const VectorLike& obstacles,
                                                                          const IPolygonalRobotShape& shape,
                                                                          DefaultClearanceCost& clearance) const
  {
    rtcus_robot_shapes::Triangles original_triangles;
    double radio;
    rtcus_robot_shapes::getTriangleBufferFromPolygon(shape, original_triangles, radio);

    float linear_dist = 0;
    clearance.linear_collision_distance_ = 0;
    VectorLike local_obstacles(obstacles.size());
    filter_radius(obstacles, local_obstacles, trajectory[0], radio);
    bool collision = this->collision_checker_->detectCollision(local_obstacles, original_triangles);
    PointXY origin;
    origin.x = 0;
    origin.y = 0;

    for (unsigned int i = 1; !collision && linear_dist < this->max_distance_ && i < trajectory.size(); i++)
    {
      filter_radius(obstacles, local_obstacles, trajectory[i], radio);
      linear_dist += sqrt(
          pow(trajectory[i - 1].x - trajectory[i].x, 2) + pow(trajectory[i - 1].y - trajectory[i].y, 2));

      if (local_obstacles.size() > 0)
      {
        //TODO: Use an enumerator like method to do a filtering of obstacles out of (trajectory + robot radious)
        rtcus_compositions::StateComposer::inverse_compose(local_obstacles, trajectory[i], local_obstacles);
        this->collision_checker_->setCircularRegionFilter(origin, radio);
        collision = this->collision_checker_->detectCollision(local_obstacles, original_triangles);
      }
    }

    if (collision)
      clearance.linear_collision_distance_ = linear_dist;
    else
      clearance.linear_collision_distance_ = std::numeric_limits<double>::max(); //NO COLLISION

    return collision;
  }

template<typename T>
  inline bool DiscreteCollisionShapedRobotClearance::computeClerance_aux(const vector<Pose2D>& trajectory,
                                                                         const T& obstacles,
                                                                         const IPolygonalRobotShape& shape,
                                                                         DefaultClearanceCost& clearance) const
  {
    RTCUS_ASSERT(trajectory.size() > 0);
    float linear_dist = 0;
    clearance.linear_collision_distance_ = 0;
    double radio;
    rtcus_robot_shapes::Triangles original_triangles;
    rtcus_robot_shapes::getTriangleBufferFromPolygon(shape, original_triangles, radio);

    bool collision = this->collision_checker_->detectCollision(obstacles, original_triangles);
    rtcus_robot_shapes::Triangles triangles(original_triangles.size());
    {
      for (unsigned int i = 1; !collision && linear_dist < this->max_distance_ && i < trajectory.size(); i++)
      {
        //TODO: Use an enumerator like method to do a filtering of obstacles out of (trajectory + robot radious)
        //rtcus_compositions::StateComposer::inverse_compose(obstacles, trajectory[i], local_obstacles);
        rtcus_compositions::StateComposer::compose((std::vector<PointXY>&)original_triangles, trajectory[i],
                                                   (std::vector<PointXY>&)triangles);

        PointXY center;
        center.x = trajectory[i].x;
        center.y = trajectory[i].y;
        this->collision_checker_->setCircularRegionFilter(center, radio);
        collision = this->collision_checker_->detectCollision(obstacles, triangles);
        linear_dist += sqrt(
            pow(trajectory[i - 1].x - trajectory[i].x, 2) + pow(trajectory[i - 1].y - trajectory[i].y, 2));
      }
    }

    if (collision)
    {
      clearance.linear_collision_distance_ = linear_dist;
      clearance.collision = true;
    }
    else
    {
      clearance.linear_collision_distance_ = std::numeric_limits<double>::max(); //NO COLLISION
      clearance.collision = false;
    }

    return collision;
  }

bool DiscreteCollisionShapedRobotClearance::computeClearance(const vector<Pose2D>& trajectory,
                                                             const pcl::PointXY& obstacles,
                                                             const IPolygonalRobotShape& shape,
                                                             DefaultClearanceCost& clearance) const
{

  return computeClerance_aux(trajectory, obstacles, shape, clearance);
}

bool DiscreteCollisionShapedRobotClearance::computeClearance(const vector<Pose2D>& trajectory,
                                                             const pcl::PointCloud<pcl::PointXY>& obstacles,
                                                             const IPolygonalRobotShape& shape,
                                                             DefaultClearanceCost& clearance) const
{
  if (strategy == 0)
    return computeClerance_aux(trajectory, obstacles, shape, clearance);
  else
    return computeClerance_auxb(trajectory, obstacles.points, shape, clearance);
}

bool DiscreteCollisionShapedRobotClearance::computeClearance(const vector<Pose2D>& trajectory,
                                                             const vector<pcl::PointXY>& obstacles,
                                                             const IPolygonalRobotShape& shape,
                                                             DefaultClearanceCost& clearance) const
{
  if (strategy == 0)
    return computeClerance_aux(trajectory, obstacles, shape, clearance);
  else
    return computeClerance_auxb(trajectory, obstacles, shape, clearance);
}

}
}

