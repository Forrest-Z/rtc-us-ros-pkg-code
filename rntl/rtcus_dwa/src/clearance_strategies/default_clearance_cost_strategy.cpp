/*
 * default_clearance_cost_strategy.cpp
 *
 *  Created on: Oct 8, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */
#include <rtcus_dwa/clearance_strategies/default_clearance_cost_strategy.h>
#include <boost/math/distributions/logistic.hpp>
#include <rtcus_assert/rtcus_assert.h>

namespace rtcus_dwa
{
using namespace std;

template<typename ObstacleInfoType>
  DefaultClearanceCostStrategy<ObstacleInfoType>::DefaultClearanceCostStrategy() :
      ObjectInfoDefaultClerance<ObstacleInfoType>::ObjectInfoDefaultClerance()
  {

  }

template<typename ObstacleInfoType>
  DefaultClearanceCostStrategy<ObstacleInfoType>::~DefaultClearanceCostStrategy()
  {

  }



template class DefaultClearanceCostStrategy<ObstacleCollisionInfo> ;
//template class DefaultClearanceCostStrategy<BasicObstacleCollisionInfo> ;

}

