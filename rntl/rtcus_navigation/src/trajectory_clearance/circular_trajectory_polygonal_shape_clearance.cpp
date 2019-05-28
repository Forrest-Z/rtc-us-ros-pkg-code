/*
 * circular_trajectory_polygonal_shape_clearance.h
 *
 *  Created on: Feb 20, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */
#include <rtcus_navigation/trajectory_clearance/circular_trajectory_polygonal_shape_clearance.h>

namespace rtcus_navigation
{
namespace trajectory_clearance
{
using namespace rtcus_nav_msgs;
using namespace rtcus_robot_shapes::interfaces;

CircularTrajectoryPolygonalRobotClearance::~CircularTrajectoryPolygonalRobotClearance()
{
  CGAL::Segment_2<CGAL::Cartesian<float> > robot_edge;
  CGAL::Segment_2<CGAL::Cartesian<float> > path_section;

  CGAL::do_intersect(robot_edge, path_section);
  vector<pcl::PointXY> triangle_buffer;

}

bool CircularTrajectoryPolygonalRobotClearance::computeClearance(const Twist2D& action, const pcl::PointXY& obstacle,
                                                                 const IPolygonalRobotShape& shape,
                                                                 double& distance) const
{
  throw std::runtime_error("not implemented");
  return false;
}

}
}
