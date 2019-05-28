/*
 * non_holonomic_kinodynamics.h
 *
 *  Created on: Feb 24, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *
 */

#ifndef NON_HOLONOMIC_KINODYNAMICS_H_
#define NON_HOLONOMIC_KINODYNAMICS_H_

#include <rtcus_navigation/kinodynamic_models/default_static_robot_kinodynamics.h>
#include <rtcus_navigation/kinodynamic_models/configurable_static_robot_kinodynamics.h>
#include <rtcus_kinodynamic_description/NonHolonomicKinoDynamicsConfig.h>

namespace rtcus_navigation
{
namespace kinodynamic_models
{

typedef KinoDynamicModel<rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig> NonHolonomicKinodynamicModel;
typedef DefaultKinodynamicModel<rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig> DefaultNonHolonomicKinodynamicModel;
typedef ConfigurableKinodynamicModel<rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig> ConfigurableNonHolonomicKinodynamicModel;
}

}

#endif /* NON_HOLONOMIC_KINODYNAMICS_H_ */
