/*
 * static_poligonal_reconfigurable.cpp
 *
 *  Created on: Dec 9, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_kinodynamic_description/NonHolonomicKinoDynamicsConfig.h>
#include <rtcus_navigation/kinodynamic_models/default_static_robot_kinodynamics.h>
#include <rtcus_navigation/kinodynamic_models/configurable_static_robot_kinodynamics.h>

#include <boost/make_shared.hpp>

namespace rtcus_navigation
{
namespace kinodynamic_models
{
using namespace rtcus_kinodynamic_description;
using namespace rtcus_navigation::kinodynamic_models;

template class ConfigurableKinodynamicModel<NonHolonomicKinoDynamicsConfig> ;
template class DefaultKinodynamicModel<NonHolonomicKinoDynamicsConfig> ;
}
}

