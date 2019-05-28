/*
 * circular_interface.h
 *
 *  Created on: Dec 9, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef CIRCULAR_INTERFACE_H_
#define CIRCULAR_INTERFACE_H_

namespace rtcus_robot_shapes
{
namespace interfaces
{
class ICircularRobotShape
{
public:
  virtual ~ICircularRobotShape()
  {
  }
  virtual double getRadius() const=0;
  virtual void setRadius(double value)=0;
};
}
}

#endif /* CIRCULAR_INTERFACE_H_ */
