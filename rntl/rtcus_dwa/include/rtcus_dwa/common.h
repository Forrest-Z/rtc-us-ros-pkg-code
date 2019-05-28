/*
 * common.h
 *
 *  Created on: Oct 8, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef COMMON_DWA_H_
#define COMMON_DWA_H_

#define DWA_MAXIMUM_RADIOUS_TO_AVOID_NUMERICAL_INESTABILITIES  0.002
#define TWO_PI 2.0*M_PI
typedef double t_float;
#include <rtcus_navigation/collision_checkers/collision_checker_pointcloud2D_circular_robot.h>
#include <rtcus_navigation/collision_checkers/collision_cheker_polygonal_robot.h>

namespace rtcus_dwa
{

//DO NOT CHANGE THE TYPE OF THIS TO DOUBLE
//THE INFORMATION TYPICALLY COMES FROM A PCL::POINTCLOUD<POINTXY> wich uses floats
typedef pcl::PointXY PointXY;
//typedef std::vector<PointXY, Eigen::aligned_allocator<PointXY> > DwaObstacleVector;
//typedef std::vector<PointXY> DwaObstacleVector;
typedef pcl::PointCloud<PointXY> DwaObstacleVector;

//typedef rtcus_navigation::collision_checkers::CollisionChekerPointCloud2DForCircularRobot CollisionCheker;
typedef rtcus_navigation::collision_checkers::CollisionChekerPolygonalRobot CollisionCheker;

template<typename GoalType>
  class DwaLocalPlannerBase;
}

#endif /* COMMON_DWA_H_ */
