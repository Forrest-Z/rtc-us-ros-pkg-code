/*
 * dwa_collision_detection.h
 *
 *  Created on: Sep 17, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef DWA_COLLISION_DETECTION_H_
#define DWA_COLLISION_DETECTION_H_

#include <rtcus_navigation/common.h>
#include <rtcus_navigation/collision_checkers/collision_checker_pointxy_circular_robot.h>

namespace rtcus_navigation
{
namespace collision_checkers
{
using namespace rtcus_robot_shapes;

/*Define the base type is required for covariance*/
typedef rtcus_navigation::CollisionChecker<pcl::PointCloud<pcl::PointXY>,
    rtcus_robot_shapes::interfaces::ICircularRobotShape> TCollisionCheckerBase;

/* TODO: This collision checker could be much more independent defining a generic interface about a circular robot.
 * For instance, a polygonal robot could be used based on its circunscribed radius.
 */
class CollisionChekerPointCloud2DForCircularRobot : public rtcus_navigation::CollisionChecker<pcl::PointCloud<pcl::PointXY>,
    rtcus_robot_shapes::interfaces::ICircularRobotShape>

{

public:
  /*Required to be a covariantType type*/
  typedef TCollisionCheckerBase TCollisionChecker;
  rtcus_navigation::collision_checkers::CollisionChekerPoint2DForCircularRobot internal_point_collision_checker_;

  CollisionChekerPointCloud2DForCircularRobot();
  virtual ~CollisionChekerPointCloud2DForCircularRobot();
  virtual void init();
  virtual void reset();
  virtual bool detectCollision(const pcl::PointCloud<pcl::PointXY> & obstacles,
                               const rtcus_robot_shapes::interfaces::ICircularRobotShape& robot_shape) const;

};
}
}
#endif /* DWA_COLLISION_DETECTION_H_ */
