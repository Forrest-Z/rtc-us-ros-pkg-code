/*
 * circular_robot_shape.cpp
 *
 *  Created on: Dec 8, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_assert/rtcus_assert.h>
#include <rtcus_robot_shapes/circular_robot.h>

namespace rtcus_robot_shapes
{

CircularRobot::CircularRobot(const CircularRobot& copy)
{
  this->setConfig(copy);
}
CircularRobot::CircularRobot(const CircularRobotShapeConfig& copy)
{
  this->setConfig(copy);
}

CircularRobot::CircularRobot()
{
}

CircularRobot::~CircularRobot()
{
}

double CircularRobot::getRadius() const
{
  return this->radius;
}

void CircularRobot::setRadius(double value)
{
  RTCUS_ASSERT_MSG(this->radius >= 0, "The robot circle radius should be >=0");
  this->radius = value;
}

const CircularRobotShapeConfig& CircularRobot::setConfig(const CircularRobotShapeConfig& config)
{
  this->setRadius(config.radius);
  return *this;
}

std::ostream& operator <<(std::ostream &out, const CircularRobot &circular_shape)
{
  return out << "[Circular robot radius: " << circular_shape.radius << "]";
}
}

