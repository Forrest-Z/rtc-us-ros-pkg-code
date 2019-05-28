/*
 *
 *  Created on: Feb 19, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef ARBITRARYTRAJECTORYCLEARANCE_SHAPED_ROBOT_H_
#define ARBITRARYTRAJECTORYCLEARANCE_SHAPED_ROBOT_H_

#include <rtcus_navigation/trajectory_clearance/trajectory_clearance.h>
#include <rtcus_robot_shapes/interfaces/polygonal.h>
#include <rtcus_navigation/collision_checkers/collision_checker_triangles_robot.h>
#include <rtcus_navigation/trajectory_clearance/default_clearance_cost.h>
#include <rtcus_nav_msgs/Pose2D.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace rtcus_navigation
{
namespace trajectory_clearance
{
using namespace rtcus_robot_shapes::interfaces;
using namespace std;
using namespace rtcus_nav_msgs;
using namespace boost;

//TODO: c++ Concepts would be great here for the robot shape. But Interface programing its also ok
class DiscreteCollisionShapedRobotClearance : public TrajectoryClearance<vector<Pose2D>, vector<pcl::PointXY>,
                                                  IPolygonalRobotShape, DefaultClearanceCost>,
                                              public TrajectoryClearance<vector<Pose2D>, pcl::PointCloud<pcl::PointXY>,
                                                  IPolygonalRobotShape, DefaultClearanceCost>,
                                              public TrajectoryClearance<vector<Pose2D>, pcl::PointXY,
                                                  IPolygonalRobotShape, DefaultClearanceCost>
{
protected:
  double max_distance_;
  shared_ptr<rtcus_navigation::collision_checkers::CollisionChekerTriangles> collision_checker_;

  template<typename T>
    inline bool computeClerance_aux(const vector<Pose2D>& trajectory, const T& obstacles,
                                    const IPolygonalRobotShape& shape, DefaultClearanceCost& clearance) const;

  template<typename T>
    inline bool computeClerance_auxb(const vector<Pose2D>& trajectory, const T& obstacles,
                                     const IPolygonalRobotShape& shape, DefaultClearanceCost& clearance) const;

public:
  int strategy;
  DiscreteCollisionShapedRobotClearance();
  DiscreteCollisionShapedRobotClearance(
      shared_ptr<rtcus_navigation::collision_checkers::CollisionChekerTriangles> collision_checker);

  virtual ~DiscreteCollisionShapedRobotClearance();
  virtual void setMaxDistance(double distance);
  virtual double getMaxDistance() const;

  /*
   //memory alignment is not required for 2D points
   virtual bool computeClearance(const vector<Pose2D>, const pcl::PointXY& obstacle, const IPolygonalRobotShape& shape,
   double& clearance) const;
   */
  virtual bool computeClearance(const vector<Pose2D>& trajectory, const vector<pcl::PointXY>& obstacles,
                                const IPolygonalRobotShape& shape, DefaultClearanceCost& clearance) const;

  virtual bool computeClearance(const vector<Pose2D>& trajectory, const pcl::PointCloud<pcl::PointXY>& obstacles,
                                const IPolygonalRobotShape& shape, DefaultClearanceCost& clearance) const;

  virtual bool computeClearance(const vector<Pose2D>& trajectory, const pcl::PointXY& obstacles,
                                const IPolygonalRobotShape& shape, DefaultClearanceCost& clearance) const;
}
;
}
}

#endif /* ARBITRARYTRAJECTORYCLEARANCECIRCULARROBOT_H_ */
