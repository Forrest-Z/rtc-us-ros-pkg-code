/*
 * polygonal_interface.h
 *
 *  Created on: Dec 9, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef POLYGONAL_INTERFACE_H_
#define POLYGONAL_INTERFACE_H_
#include <vector>
#include <pcl/point_types.h>

namespace rtcus_robot_shapes
{
namespace interfaces
{
using namespace std;
class IPolygonalRobotShape
{
public:
  virtual ~IPolygonalRobotShape()
  {
  }
  virtual void setPoints(const vector<pcl::PointXY>& points)=0;
  virtual const vector<pcl::PointXY>& getPoints() const=0;
};
}
}

#endif /* POLYGONAL_INTERFACE_H_ */
