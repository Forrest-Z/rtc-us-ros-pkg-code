/*
 * polygonal_reconfigurable_robot_shape_model.cpp
 *
 *  Created on: Dec 19, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_navigation/shape_models/polygonal_configurable_shape_model.h>
namespace rtcus_navigation
{
namespace shape_models
{

void PolygonalConfigurableRobotShapeModel::onStep(const std::string& base_frame)
{
  this->circumscribed_rosmsg_.header.stamp = this->polygon_msg_.header.stamp = ros::Time::now();
  this->circumscribed_rosmsg_.header.frame_id = this->polygon_msg_.header.frame_id = base_frame;

  this->robot_shape_pub_.publish(this->polygon_msg_);
  this->robot_radious_pub_.publish(this->circumscribed_rosmsg_);
}

PolygonalConfigurableRobotShapeModel::PolygonalConfigurableRobotShapeModel()
{
  ros::NodeHandle& nh = this->component_node_;
  robot_shape_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("polygonal_shape", 1.0, true);
  robot_radious_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("circumscribed_circle", 1.0, true);

}

PolygonalConfigurableRobotShapeModel::~PolygonalConfigurableRobotShapeModel()
{
}

void PolygonalConfigurableRobotShapeModel::setRobotShape(const rtcus_robot_shapes::PolygonalRobot& shape)
{
  ConfigurableRobotShapeModel<rtcus_robot_shapes::PolygonalRobot, ROSTimeModel>::setRobotShape(shape);
  shape.getROSPolygon(this->polygon_msg_.polygon);
  this->circumscribed_rosmsg_.polygon.points.clear();
  for (float i = 0; i < 2 * M_PI; i += 0.4)
  {
    geometry_msgs::Point32 p;
    p.x = shape.getRadius() * cos(i);
    p.y = shape.getRadius() * sin(i);
    this->circumscribed_rosmsg_.polygon.points.push_back(p);
  }

  geometry_msgs::Point32 p1 = this->circumscribed_rosmsg_.polygon.points[0];
  this->circumscribed_rosmsg_.polygon.points.push_back(p1);

}
}
}

