/*
 * circular_trajectory_polygonal_shape_clearance.h
 *
 *  Created on: Feb 20, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */
#include <rtcus_navigation/trajectory_clearance/trajectory_clearance.h>
#include <rtcus_nav_msgs/Twist2D.h>
#include <rtcus_robot_shapes/interfaces/polygonal.h>
#include <pcl/point_types.h>

namespace rtcus_navigation
{
namespace trajectory_clearance
{
using namespace rtcus_nav_msgs;
using namespace rtcus_robot_shapes::interfaces;

class CircularTrajectoryPolygonalRobotClearance :
    public TrajectoryClearance<Twist2D, pcl::PointXY, IPolygonalRobotShape>
{

public:
  virtual ~CircularTrajectoryPolygonalRobotClearance();
  virtual bool computeClearance(const Twist2D& action, const pcl::PointXY& obstacle, const IPolygonalRobotShape& shape,
                                double& distance) const;
  virtual double getMaxDistance() const;
};
}
}

