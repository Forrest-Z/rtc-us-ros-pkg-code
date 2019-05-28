/*
 * static_poligonal_reconfigurable.cpp
 *
 *  Created on: Dec 9, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_robot_shapes/polygonal_robot.h>
#include <rtcus_navigation/shape_models/default_static_robot_shape.h>
#include <rtcus_navigation/shape_models/configurable_static_robot_shape.h>
#include <boost/make_shared.hpp>

namespace rtcus_navigation
{
namespace shape_models
{

using namespace rtcus_robot_shapes;

template class ConfigurableRobotShapeModel<PolygonalRobot, ROSTimeModel> ;
template class DefaultStaticRobotShape<PolygonalRobot, ROSTimeModel> ;
}
}

