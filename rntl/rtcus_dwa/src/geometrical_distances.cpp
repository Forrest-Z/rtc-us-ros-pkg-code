/*
 * geometrical_distances.cpp
 *
 *  Created on: Oct 3, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_dwa/collision_distance_methods.h>
#include <math.h>
#include <ros/ros.h>
#include <rtcus_assert/rtcus_assert.h>
#include <rtcus_dwa/common.h>

namespace rtcus_dwa
{
using namespace std;
void proyectObstacleToCircularTrajectory(double v, double omega, double px, double py, double& separation_1,
                                         double& linear_distance_1, PointXY& normal_point_1, double& separation_2,
                                         double& linear_distance_2, PointXY& normal_point_2)
{
  if (fabs(omega) > DWA_MAXIMUM_RADIOUS_TO_AVOID_NUMERICAL_INESTABILITIES)
  {
    bool flip = omega < 0;
    double radio;
    if (flip)
    {
      omega = -omega;
      py = -py;
      radio = fabs(v / omega);
    }
    else
      radio = v / omega;

    double icr_y = radio;
    double diff_y = (py - icr_y);
    //UNDERSTAND THIS: this is the distance from the ICR to P
    double icr_dist = sqrt(px * px + diff_y * diff_y);

    double vx = px / icr_dist;
    double vy = diff_y / icr_dist;

    //solution 1
    //the nearess in linear distance
    double delta_angle_1;
    {
      normal_point_1.x = 0.0 + radio * vx;
      normal_point_1.y = icr_y + radio * vy;
      delta_angle_1 = atan2(normal_point_1.x, icr_y - normal_point_1.y);

      while (delta_angle_1 < 0.0)
        delta_angle_1 += 2.0 * M_PI;
      linear_distance_1 = delta_angle_1 * fabs(radio);
      separation_1 = sqrt(pow(normal_point_1.x - px, 2) + pow(normal_point_1.y - py, 2));
    }

    //solution 2
    //the further at linear distance

    double delta_angle_2;
    {
      normal_point_2.x = 0.0 - radio * vx;
      normal_point_2.y = icr_y - radio * vy;
      delta_angle_2 = atan2(normal_point_2.x, icr_y - normal_point_2.y);

      while (delta_angle_2 < 0.0)
        delta_angle_2 += 2.0 * M_PI;
      linear_distance_2 = delta_angle_2 * fabs(radio);
      separation_2 = sqrt(pow(normal_point_2.x - px, 2) + pow(normal_point_2.y - py, 2));
    }

    RTCUS_ASSERT(separation_1 <= separation_2);

    //  ROS_INFO(
    //      "values l1 %lf a1 %lf(nx %lf ny %lf)|| l2 %lf a2 %lf (nx %lf ny %lf)||(flip: %d , obstacle(%lf %lf) , ICRY: %lf, radious %lf)", linear_distance_1, delta_angle_1*180.0/M_PI, normal_point_1.x, normal_point_1.y, linear_distance_2, delta_angle_2*180.0/M_PI, normal_point_2.x, normal_point_2.y, flip, px, py, icr_y, radio);

    if (flip)
    {
      normal_point_1.y = -normal_point_1.y;
      normal_point_2.y = -normal_point_2.y;
      py = -py;
    }

  }
  else
  {
    separation_1 = separation_2 = fabs(py);

    normal_point_1.x = px;
    normal_point_2.x = px;
    normal_point_1.y = 0;
    normal_point_2.y = 0;
    if (v >= 0 && px < 0)
    {
      linear_distance_1 = numeric_limits<double>::infinity();
      linear_distance_2 = numeric_limits<double>::infinity();
    }
    else
    {
      linear_distance_1 = px;
      linear_distance_2 = px;
    }
  }
}

}
