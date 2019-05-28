/*
 * separation_obstacle_ponderated_clearance_cost_strategy.cpp
 *
 *  Created on: Oct 10, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_dwa/clearance_strategies/separation_obstacle_cost_strategy.h>
#include <boost/math/distributions/logistic.hpp>
#include <rtcus_assert/rtcus_assert.h>

namespace rtcus_dwa
{
/*template class ObstacleSeparationPonderatedClearanceCostStrategy<double> ;
 template class ObstacleSeparationPonderatedClearanceCostStrategy<float> ;

 template<typename t_float>
 t_float ObstacleSeparationPonderatedClearanceCostStrategy<t_float>::getNormalRepulsionObstacleCost(
 float separation_distance, float linear_distance) const
 {
 boost::math::logistic logistic(this->config_->clearance_sigmoid_location, this->config_->clearance_sigmoid_scale);
 double normalization_more_distant_collision_distance = this->normalize();

 if (linear_distance < normalization_more_distant_collision_distance)
 {
 double nearness_cost = boost::math::cdf(logistic,
 (linear_distance / normalization_more_distant_collision_distance));

 double separation_cost = boost::math::cdf(logistic,
 separation_distance / this->config_->security_area_obstacle_inflation);
 return nearness_cost;
 }

 else
 return 0.0;
 }
 */

}

