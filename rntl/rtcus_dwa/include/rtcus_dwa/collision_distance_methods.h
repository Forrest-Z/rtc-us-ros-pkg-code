/*
 * collision_distance_methods.h
 *
 *  Created on: Oct 3, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef COLLISION_DISTANCE_METHODS_H_
#define COLLISION_DISTANCE_METHODS_H_

#include <rtcus_dwa/common.h>

namespace rtcus_dwa
{

/*
 enum DwaObstacleDistanceResult
 {
 DISTANCE_COMPUTED = 0, INSIDE_OBSTACLE = -1, NO_COLLISION_FOUND = 1000, INTERNAL_LOOP = -2
 };

 DwaObstacleDistanceResult circular_trajectory_collision(double v, double omega, double ox, double oy,
 double obstacle_radious, double max_collision_distance,
 bool forward, t_float &distance, t_float& second_distance,
 PointXY & nearest_collision_point,
 PointXY& furtherst_collision_point);
 */

void proyectObstacleToCircularTrajectory(double v, double omega, double px, double py, double& separation_1,
                                         double& linear_distance_1, PointXY& normal_point_1, double& separation_2,
                                         double& linear_distance_2, PointXY& normal_point_2);

}

#endif /* COLLISION_DISTANCE_METHODS_H_ */
