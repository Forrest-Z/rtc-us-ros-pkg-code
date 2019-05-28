/*
 * circular_trayectory_for_circular_robot_clearance.h
 *
 *  Created on: Dec 10, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef CIRCULAR_TRAYECTORY_FOR_CIRCULAR_ROBOT_CLEARANCE_H_
#define CIRCULAR_TRAYECTORY_FOR_CIRCULAR_ROBOT_CLEARANCE_H_

#include <rtcus_navigation/trajectory_clearance/trajectory_clearance.h>
#include <rtcus_nav_msgs/Twist2D.h>
#include <rtcus_robot_shapes/circular_robot.h>
#include <rtcus_navigation/trajectory_clearance/default_clearance_cost.h>

//obstacles
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>

namespace rtcus_navigation
{
namespace trajectory_clearance
{
using namespace rtcus_robot_shapes::interfaces;
using namespace rtcus_nav_msgs;
using namespace std;

/**
 * \brief To avoid numerical instabilities in during the divisions algorithm
 * */
const double MIN_DIVISION_DENOMINATOR = 0.002;

/**
 * \brief vectorial trajectory collision checking for a circular trajectory of a circular robot with an obstacle point.
 * \remarks This implementation does not take into account clearance backwards (still TODO)
 * */
class CircularTrajectoryClearanceForCircularRobot : public TrajectoryClearance<Twist2D, pcl::PointXY,
                                                        ICircularRobotShape, DefaultClearanceCost>,
                                                    public TrajectoryClearance<Twist2D, pcl::PointCloud<pcl::PointXY>,
                                                        ICircularRobotShape, DefaultClearanceCost>,
                                                    public TrajectoryClearance<Twist2D, std::vector<pcl::PointXY>,
                                                        ICircularRobotShape, DefaultClearanceCost>
{
public:

  typedef enum
  {
    COLLISION = 0, INSIDE_OBSTACLE = -1, NO_COLLISION_FOUND = 1000, INTERNAL_LOOP = -2
  } ClearanceResult;

  typedef double t_float;
protected:
  inline bool computeClearance_aux(const Twist2D& action, const pcl::PointXY& obstacle,
                                   const ICircularRobotShape& shape, DefaultClearanceCost& distance);

  template<typename Titerator>
    inline bool computeClearance_auxit(Titerator itbegin, Titerator itend, const Twist2D& action,
                                       const ICircularRobotShape& shape, DefaultClearanceCost& distance);

  inline ClearanceResult computeClearance_auxb(const Twist2D& action, const pcl::PointXY& obstacle, double robot_radius,
                                               double max_collision_distance, bool forward, t_float &distance,
                                               t_float& second_distance, pcl::PointXY & nearest_collision_point,
                                               pcl::PointXY& furtherst_collision_point,
                                               t_float obstacle_inflation) const;

public:

  CircularTrajectoryClearanceForCircularRobot();
  virtual ~CircularTrajectoryClearanceForCircularRobot();

  t_float second_distance_;
  pcl::PointXY nearest_collision_point_;
  pcl::PointXY furtherst_collision_point_;
  t_float max_distance_;
  bool forward_;
  ClearanceResult result_;
  float obstacle_inflation_;
  bool compute_second_point_;

  /**
   * \brief general clearance method
   * */
  virtual bool computeClearance(const Twist2D& action, const pcl::PointXY& obstacle, const ICircularRobotShape& shape,
                                DefaultClearanceCost& distance) const;

  virtual bool computeClearance(const Twist2D& action, const pcl::PointXY& obstacle, const ICircularRobotShape& shape,
                                DefaultClearanceCost& distance);

  ClearanceResult computeClearance(const Twist2D& action, const pcl::PointXY& obstacle, double robot_radius,
                                   double max_collision_distance, bool forward, t_float &distance,
                                   t_float& second_distance, pcl::PointXY & nearest_collision_point,
                                   pcl::PointXY& furtherst_collision_point, t_float obstacle_inflation) const;

  virtual bool computeClearance(const Twist2D& action, const std::vector<pcl::PointXY>& obstacle,
                                const ICircularRobotShape& shape, DefaultClearanceCost& distance) const;

  virtual bool computeClearance(const Twist2D& action, const pcl::PointCloud<pcl::PointXY>& obstacle,
                                const ICircularRobotShape& shape, DefaultClearanceCost& distance) const;

  virtual void setMaxDistance(double distance);
  virtual double getMaxDistance() const;

};
}
}

#endif /* CIRCULAR_TRAYECTORY_FOR_CIRCULAR_ROBOT_CLEARANCE_H_ */
