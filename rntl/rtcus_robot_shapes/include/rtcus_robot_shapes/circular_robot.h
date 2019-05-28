/*
 * circular_robot.h
 *
 *  Created on: Dec 9, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef CIRCULAR_ROBOT_H_
#define CIRCULAR_ROBOT_H_

#include <rtcus_robot_shapes/interfaces/circular.h>
#include <rtcus_robot_shapes/CircularRobotShapeConfig.h>
#include <rtcus_robot_shapes/configurable_shape.h>

namespace rtcus_robot_shapes
{

class CircularRobot : public interfaces::ICircularRobotShape, public ConfigurableShape<CircularRobotShapeConfig>
{
public:
  CircularRobot();
  CircularRobot(const CircularRobot& copy);
  CircularRobot(const CircularRobotShapeConfig& copy);
  virtual ~CircularRobot();
  virtual double getRadius() const;
  virtual void setRadius(double value);
  virtual const CircularRobotShapeConfig& setConfig(const CircularRobotShapeConfig& config);
  friend std::ostream& operator<<(std::ostream &out, const CircularRobot&circular_shape);
};

}

#endif /* CIRCULAR_ROBOT_H_ */
