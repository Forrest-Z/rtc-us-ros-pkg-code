/*
 * obstacle_collision_info.cpp
 *
 *  Created on: Oct 9, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_dwa/clearance_strategies/obstacle_collision_info.h>
#include <limits>
#include <rtcus_assert/rtcus_assert.h>

namespace rtcus_dwa
{

ObstacleCollisionInfo::ObstacleCollisionInfo()
{
  init();
}

ObstacleCollisionInfo::~ObstacleCollisionInfo()
{

}

void ObstacleCollisionInfo::init()
{
  BasicObstacleCollisionInfo::init();
  is_dangerous_obstacle_ = false;
  proyection_collision_separation_ = std::numeric_limits<t_float>::quiet_NaN();
  proyection_collision_distance_ = std::numeric_limits<t_float>::quiet_NaN();
  normal_repulsion_cost_ = std::numeric_limits<t_float>::quiet_NaN();

  danger_area_entering_distance = std::numeric_limits<t_float>::quiet_NaN();
  danger_area_leaving_distance = std::numeric_limits<t_float>::quiet_NaN();

  danger_area_internal_loop_ = false;
}

void ObstacleCollisionInfo::setIsDangerousObstacle()
{
  is_dangerous_obstacle_ = true;
}

bool ObstacleCollisionInfo::isDangerousObstacle() const
{
  return is_dangerous_obstacle_;
}

void ObstacleCollisionInfo::setCollision(double collision_distance, const PointXY& collision_point)
{
  this->setCollisionDistance(collision_distance);
  collision_point_ = collision_point;
}

const PointXY& ObstacleCollisionInfo::getCollisionCoordinates() const
{
  RTCUS_ASSERT(this->hasTrajectoryCollision());
  return this->collision_point_;
}

//----------- PRIMARY PROYECTION ---------------

void ObstacleCollisionInfo::setCollisionProyectionPoint(const PointXY& obstacle_proyection_point,
                                                        double collision_separation, double collision_linear_distance,
                                                        double normal_repulsion_cost)
{
  RTCUS_ASSERT(!this->hasTrajectoryCollision());
  this->proyection_collision_point = obstacle_proyection_point;
  this->proyection_collision_separation_ = collision_separation;
  this->proyection_collision_distance_ = collision_linear_distance;
  this->normal_repulsion_cost_ = normal_repulsion_cost;
}

bool ObstacleCollisionInfo::hasProyectedObstacle() const
{
  return proyection_collision_distance_ == proyection_collision_distance_;
}

t_float ObstacleCollisionInfo::getSeparationDistance() const
{
  RTCUS_ASSERT_MSG(
      !this->hasTrajectoryCollision() && hasProyectedObstacle(),
      "The current trajectory is colliding %lf with this obstacle ox %lf oy %lf. But the Collision proyection distace (%lf) has been requested.", collision_distance_, this->coordinates.x, this->coordinates.y, proyection_collision_distance_);
  return proyection_collision_separation_;
}

double ObstacleCollisionInfo::getNormalRepulsionCost() const
{
  RTCUS_ASSERT_MSG(
      !this->hasTrajectoryCollision() && hasProyectedObstacle(),
      "The current trajectory is colliding %lf with this obstacle ox %lf oy %lf. But the Collision proyection distace (%lf) has been requested.", collision_distance_, this->coordinates.x, this->coordinates.y, proyection_collision_distance_);
  return this->normal_repulsion_cost_;
}

t_float ObstacleCollisionInfo::getLinearProyectedDistance() const
{
  RTCUS_ASSERT_MSG(
      !this->hasTrajectoryCollision() && hasProyectedObstacle(),
      "The current trajectory is colliding %lf with this obstacle ox %lf oy %lf. But the Collision proyection distace (%lf) has been requested.", collision_distance_, this->coordinates.x, this->coordinates.y, proyection_collision_distance_);
  return this->proyection_collision_distance_;
}

const PointXY& ObstacleCollisionInfo::getProyectedCoordinates() const
{
  RTCUS_ASSERT_MSG(
      !this->hasTrajectoryCollision() && hasProyectedObstacle(),
      "The current trajectory is colliding %lf with this obstacle ox %lf oy %lf. But the Collision proyection distace (%lf) has been requested.", collision_distance_, this->coordinates.x, this->coordinates.y, proyection_collision_distance_);
  return this->proyection_collision_point;
}

//----------- SECONDARY PROYECTION ---------------

void ObstacleCollisionInfo::setSecondaryCollisionProyectionPoint(const PointXY& obstacle_proyection_point,
                                                                 double collision_separation,
                                                                 double collision_linear_distance)
{
  RTCUS_ASSERT(!this->hasTrajectoryCollision());
  this->secondary_proyection_collision_point = obstacle_proyection_point;
  this->secondary_proyection_collision_separation_ = collision_separation;
  this->secondary_proyection_collision_distance_ = collision_linear_distance;

}

bool ObstacleCollisionInfo::hasSecondaryProyectedObstacle() const
{
  return secondary_proyection_collision_distance_ == secondary_proyection_collision_distance_;
}

t_float ObstacleCollisionInfo::getSecondarySeparationDistance() const
{
  RTCUS_ASSERT_MSG(
      !this->hasTrajectoryCollision() && hasSecondaryProyectedObstacle(),
      "The current trajectory is colliding %lf with this obstacle ox %lf oy %lf. But the Collision proyection distace (%lf) has been requested.", collision_distance_, this->coordinates.x, this->coordinates.y, proyection_collision_distance_);
  return secondary_proyection_collision_separation_;
}

t_float ObstacleCollisionInfo::getSecondaryLinearProyectedDistance() const
{
  RTCUS_ASSERT_MSG(
      !this->hasTrajectoryCollision() && hasSecondaryProyectedObstacle(),
      "The current trajectory is colliding %lf with this obstacle ox %lf oy %lf. But the Collision proyection distace (%lf) has been requested.", collision_distance_, this->coordinates.x, this->coordinates.y, proyection_collision_distance_);
  return this->secondary_proyection_collision_distance_;
}

const PointXY& ObstacleCollisionInfo::getSecondaryProyectedCoordinates() const
{
  RTCUS_ASSERT_MSG(
      !this->hasTrajectoryCollision() && hasSecondaryProyectedObstacle(),
      "The current trajectory is colliding %lf with this obstacle ox %lf oy %lf. But the Collision proyection distace (%lf) has been requested.", collision_distance_, this->coordinates.x, this->coordinates.y, proyection_collision_distance_);
  return this->secondary_proyection_collision_point;
}

void ObstacleCollisionInfo::getGraphicalRepresentation(visualization_msgs::Marker& m) const
{
  if (this->hasTrajectoryCollision())
  {
    const PointXY& p = getCollisionCoordinates();
    m.type = visualization_msgs::Marker::SPHERE;
    m.pose.position.x = p.x;
    m.pose.position.y = p.y;
    m.pose.position.z = 0.01;
    m.pose.orientation.w = 1.0;

    m.scale.x = 0.03;
    m.scale.y = 0.03;
    m.scale.z = 0.03;

    m.color.b = 0.1;
    m.color.g = 0.1;
    m.color.r = 1.0;
    m.color.a = 1.0;
    m.ns = "collision";
  }

}
}

