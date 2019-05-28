/*
 * costmap_perception_port.cpp
 *
 *  Created on: Nov 19, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_navigation/world_perception_ports/cost_map.h>
#include <rtcus_assert/rtcus_assert.h>

namespace rtcus_navigation
{
namespace world_perception_ports
{

#define SENSOR_LINK "sensor_link_frame"
#define BASE_LINK "base_link_frame"
using namespace boost;
CostMapPerceptionPort::CostMapPerceptionPort()

{

}

CostMapPerceptionPort::~CostMapPerceptionPort()
{

}
;

void CostMapPerceptionPort::reset()
{

}

void CostMapPerceptionPort::init()
{
  std::string costmap_namespace = this->getArchitectureComponentName();
  ROS_INFO( "Initializating Costmap perception port in the namespace %s", costmap_namespace.c_str());
  costmap_controller_ = boost::shared_ptr<costmap_2d::Costmap2DROS>(
      new costmap_2d::Costmap2DROS(costmap_namespace, tf_listener_));
  ROS_INFO("costmap created");
  obstacles_data_ = rtcus_stamp::Stamped<costmap_2d::Costmap2DROS>(costmap_controller_);

  if (!component_node_.getParam(SENSOR_LINK, sensor_link_frame_))
    setSensorFrameName(sensor_link_frame_);

  if (!component_node_.getParam(BASE_LINK, mobile_base_frame_))
    setMobileBaseFrameName(mobile_base_frame_);

  ROS_INFO(
      "Setting the global fixed frame for the obstacle representation: %s", costmap_controller_->getGlobalFrameID().c_str());

  this->obstacles_data_.setFrameId(costmap_controller_->getGlobalFrameID());

  RTCUS_ASSERT(ARE_SAME_FRAMES(mobile_base_frame_, costmap_controller_->getBaseFrameID()));
  ROS_INFO("Running the Costmap Thread");
  costmap_controller_->start();
  ROS_INFO("Costmap started");

}

shared_ptr<mutex> CostMapPerceptionPort::getLastUpdateWorld(Stamped<costmap_2d::Costmap2DROS>& obstacles)
{
  obstacles = this->obstacles_data_;
  //no locking protection since the obstacle type has its own concurrency protection implemented
  return boost::shared_ptr<boost::mutex>();
}

shared_ptr<mutex> CostMapPerceptionPort::getWorldEstimation(ros::Time time,
                                                            Stamped<costmap_2d::Costmap2DROS>& obstacles)
{

  return this->getLastUpdateWorld(obstacles);

}

bool CostMapPerceptionPort::hasValidObstaclesEstimation()
{
  return true;
}

void CostMapPerceptionPort::pushObstacles(const costmap_2d::Costmap2DROS& obstacles)
{
  RTCUS_ASSERT_MSG(false, "Not implemented, is this needed?");
}

void CostMapPerceptionPort::setSensorFrameName(const std::string& value)
{
  this->sensor_link_frame_ = value;
  this->component_node_.setParam(SENSOR_LINK, value);
}

std::string CostMapPerceptionPort::getSensorFrameName() const
{
  return sensor_link_frame_;
}

void CostMapPerceptionPort::setMobileBaseFrameName(const std::string& value)
{
  this->mobile_base_frame_ = value;
  this->component_node_.setParam(BASE_LINK, value);
}

std::string CostMapPerceptionPort::getMobileBaseFrameName() const
{
  return mobile_base_frame_;
}

void CostMapPerceptionPort::allocateObstacleRepresentation(rtcus_stamp::Stamped<costmap_2d::Costmap2DROS>& obstacles)
{
  //no allocation, using this own data
  obstacles = this->obstacles_data_;
  ROS_INFO("Trying to allocate the costmap data, but returned a pointer.");
  ROS_INFO("The costmap stamp is %s", this->obstacles_data_.getFrameId().c_str());
}

}
}

