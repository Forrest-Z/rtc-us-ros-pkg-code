/*
 * cost_maps.h
 *
 *  Created on: Apr 10, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef COST_MAP_H_
#define COST_MAP_H_

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <rtcus_navigation/world_perception.h>
#include <rtcus_navigation/common.h>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/thread.hpp>

namespace rtcus_navigation
{
namespace world_perception_ports
{
#define SENSOR_LINK "sensor_link_frame"
#define BASE_LINK "base_link_frame"
using namespace boost;

class CostMapPerceptionPort : public WorldPerceptionPort<costmap_2d::Costmap2DROS>

{
protected:
  shared_ptr<costmap_2d::Costmap2DROS> costmap_controller_;
  Stamped<costmap_2d::Costmap2DROS> obstacles_data_;

  tf::TransformListener tf_listener_;
  std::string sensor_link_frame_;
  std::string mobile_base_frame_;

public:
  boost::signal<void(WorldPerceptionPort<costmap_2d::Costmap2DROS>& sender)> onWorldUpdate;

  CostMapPerceptionPort();
  virtual ~CostMapPerceptionPort();

  virtual void reset();
  virtual void init();
  virtual shared_ptr<mutex> getLastUpdateWorld(Stamped<costmap_2d::Costmap2DROS>& obstacles);
  virtual shared_ptr<mutex> getWorldEstimation(ros::Time time, Stamped<costmap_2d::Costmap2DROS>& obstacles);
  virtual bool hasValidObstaclesEstimation();
  virtual void pushObstacles(const costmap_2d::Costmap2DROS& obstacles);
  virtual void setSensorFrameName(const std::string& value);
  virtual std::string getSensorFrameName() const;
  virtual void setMobileBaseFrameName(const std::string& value);
  virtual std::string getMobileBaseFrameName() const;
  virtual void allocateObstacleRepresentation(rtcus_stamp::Stamped<costmap_2d::Costmap2DROS>& obstacles);
};
}
}

#endif /* COST_MAP_H_ */
