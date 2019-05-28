/*
 * triangles.h
 *
 *  Created on: Feb 25, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef TRIANGLES_H_
#define TRIANGLES_H_
#include <vector>
#include <pcl/point_types.h>
namespace rtcus_robot_shapes
{
class Triangles : public std::vector<pcl::PointXY>
{
public:
  Triangles()
  {
  }
  Triangles(unsigned int size) :
      vector<pcl::PointXY>(size)
  {
  }
};
}

#endif /* TRIANGLES_H_ */
