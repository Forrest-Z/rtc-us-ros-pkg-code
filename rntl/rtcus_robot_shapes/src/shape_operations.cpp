/*
 * shape_operations.cpp
 *
 *  Created on: Feb 25, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */
#include <rtcus_robot_shapes/shape_operations.h>

namespace rtcus_robot_shapes
{
using namespace rtcus_robot_shapes::interfaces;

void getTriangleBufferFromPolygon(const IPolygonalRobotShape& robot_shape, Triangles& triangles, double& radio)
{
  const PolygonalRobot* shape = dynamic_cast<const PolygonalRobot*>(&robot_shape);
  if (shape != NULL)
  {
    const_cast<PolygonalRobot*>(shape)->getTriangularization(triangles);
    radio = shape->getRadius();
  }
  else
  {
    PolygonalRobot tmp;
    tmp.setPoints(robot_shape.getPoints());
    tmp.getTriangularization(triangles);
    radio = tmp.getRadius();
  }
}
}

