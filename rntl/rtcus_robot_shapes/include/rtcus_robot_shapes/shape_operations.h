/*
 * shape_operations.h
 *
 *  Created on: Feb 25, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef SHAPE_OPERATIONS_H_
#define SHAPE_OPERATIONS_H_

#include <rtcus_robot_shapes/interfaces/polygonal.h>
#include <rtcus_robot_shapes/interfaces/triangles.h>
#include <rtcus_robot_shapes/polygonal_robot.h>

namespace rtcus_robot_shapes
{
using namespace rtcus_robot_shapes::interfaces;

//TODO: move to rtcus_convert
void getTriangleBufferFromPolygon(const IPolygonalRobotShape& robot_shape, Triangles& triangles, double& radio);
}

#endif /* SHAPE_OPERATIONS_H_ */
