/*
 * obstacle_collision_info.h
 *
 *  Created on: Oct 7, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef OBSTACLE_COLLISION_INFO_H_
#define OBSTACLE_COLLISION_INFO_H_
#include <rtcus_dwa/common.h>
#include <visualization_msgs/Marker.h>

namespace rtcus_dwa
{

class BasicObstacleCollisionInfo
{
protected:
  double collision_distance_;

public:
  PointXY coordinates;
  BasicObstacleCollisionInfo()
  {

  }
  virtual ~BasicObstacleCollisionInfo()
  {
  }
  virtual void init()
  {
    collision_distance_ = std::numeric_limits<t_float>::quiet_NaN();
  }
  inline void setCollisionDistance(double collision_distance)
  {
    RTCUS_ASSERT(collision_distance >= 0);
    collision_distance_ = collision_distance;
  }

  inline bool hasTrajectoryCollision() const
  {
    return collision_distance_ == collision_distance_;
  }

  inline double getCollisionDistance() const
  {
    RTCUS_ASSERT(hasTrajectoryCollision());
    return this->collision_distance_;
  }

  virtual void getGraphicalRepresentation(visualization_msgs::Marker& m) const
  {
  }

};
//-------------------------------------------------------------------------------

typedef enum
{
  COLLISION, ENTERING_DANGER_AREA, LEAVING_DANGER_AREA, DANGER_OBSTACLE_KEY_POINT
} TCollisionKeyPointType;

/**
 * \brief This class is obsolete. This rich representation of the obstacle collision information was used initially for complex clearance algorithms.
 * However in the end, the most basic representation BasicObstacleCollisionInfo is being used for efficency and simplicity.
 * */
class ObstacleCollisionInfo : public BasicObstacleCollisionInfo
{
protected:

  PointXY collision_point_;
  bool is_dangerous_obstacle_;

public:
  PointXY danger_area_entering_point;
  PointXY danger_area_leaving_point;
  t_float danger_area_entering_distance;
  t_float danger_area_leaving_distance;

  bool danger_area_internal_loop_;

  virtual ~ObstacleCollisionInfo();
  ObstacleCollisionInfo();

  virtual void init();

  void setIsDangerousObstacle();
  bool isDangerousObstacle() const;

  void setCollision(double collision_distance, const PointXY& collision_point);
  const PointXY& getCollisionCoordinates() const;

  //--------------- primary proyection ----------------------
  t_float proyection_collision_separation_;
  t_float proyection_collision_distance_;
  PointXY proyection_collision_point;

  t_float secondary_proyection_collision_separation_;
  t_float secondary_proyection_collision_distance_;
  PointXY secondary_proyection_collision_point;

  t_float normal_repulsion_cost_;
  void setCollisionProyectionPoint(const PointXY& obstacle_proyection_point, double collision_separation,
                                   double collision_linear_distance, double cost);
  double getNormalRepulsionCost() const;
  bool hasProyectedObstacle() const;
  t_float getSeparationDistance() const;
  t_float getLinearProyectedDistance() const;
  const PointXY& getProyectedCoordinates() const;

  //--------------- secondary proyection -------------------------
  void setSecondaryCollisionProyectionPoint(const PointXY& obstacle_proyection_point, double collision_separation,
                                            double collision_linear_distance);
  bool hasSecondaryProyectedObstacle() const;
  t_float getSecondarySeparationDistance() const;
  t_float getSecondaryLinearProyectedDistance() const;
  const PointXY& getSecondaryProyectedCoordinates() const;
  virtual void getGraphicalRepresentation(visualization_msgs::Marker& m) const;
};

}

#endif /* OBSTACLE_COLLISION_INFO_H_ */
