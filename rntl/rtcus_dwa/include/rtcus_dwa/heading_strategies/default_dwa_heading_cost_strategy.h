/*
 * default_dwa_heading_cost_strategy.h
 *
 *  Created on: Oct 25, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef DEFAULT_DWA_HEADING_COST_STRATEGY_H_
#define DEFAULT_DWA_HEADING_COST_STRATEGY_H_
#include <rtcus_dwa/heading_cost_strategies.h>

namespace rtcus_dwa
{
namespace heading_strategies
{

/**
 * \brief This class implements the default heading function of the DWA algorithm specified in its original statement.
 * */
class DefaultDwaHeadingCostStrategy : public rtcus_dwa::HeadingCostStrategy<pcl::PointXY, Twist2D, Twist2D>
{

public:
  virtual t_float computeCost(const CommandCost<rtcus_nav_msgs::Twist2D>& action);
  virtual ~DefaultDwaHeadingCostStrategy()
  {
  }
};

}
}

#endif /* DEFAULT_DWA_HEADING_COST_STRATEGY_H_ */
