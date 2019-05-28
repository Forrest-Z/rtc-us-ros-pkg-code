/*
 * ConfigurablePolygonalShape.h
 *
 *  Created on: Dec 19, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef CONFIGURABLEPOLYGONALSHAPE_H_
#define CONFIGURABLEPOLYGONALSHAPE_H_
#include <rtcus_robot_shapes/polygonal_robot.h>
#include <rtcus_navigation/shape_models/configurable_static_robot_shape.h>
#include <geometry_msgs/PolygonStamped.h>
#include <boost/type_traits.hpp>

namespace rtcus_navigation
{
namespace shape_models
{
/**
 * \brief This class allow to a navigation node store a polygonal shape of the robot, while it is published periodically through a topic
 * */
class PolygonalConfigurableRobotShapeModel : public ConfigurableRobotShapeModel<rtcus_robot_shapes::PolygonalRobot,
    ROSTimeModel>
{
protected:
  ros::Publisher robot_shape_pub_;
  ros::Publisher robot_radious_pub_;
  geometry_msgs::PolygonStamped polygon_msg_;
  geometry_msgs::PolygonStamped circumscribed_rosmsg_;

  void onStep(const std::string& base_frame);

public:
  PolygonalConfigurableRobotShapeModel();
  virtual ~PolygonalConfigurableRobotShapeModel();
  virtual void setRobotShape(const rtcus_robot_shapes::PolygonalRobot& shape);

  virtual void onDecoratedRobotShapeChanged(const rtcus_robot_shapes::PolygonalRobot& shape)
  {
    ConfigurableRobotShapeModel<rtcus_robot_shapes::PolygonalRobot, ROSTimeModel>::onShapeChanged(shape);

    this->circumscribed_rosmsg_.header.stamp = this->polygon_msg_.header.stamp = ros::Time::now();
    this->robot_shape_pub_.publish(this->polygon_msg_);
    this->robot_radious_pub_.publish(this->circumscribed_rosmsg_);
  }

  template<typename NavigationNode>
    void setNavigationNode(NavigationNode& nn)
    {
      this->polygon_msg_.header.frame_id = nn.getBaseFrameName();
      this->circumscribed_rosmsg_.header.frame_id = nn.getBaseFrameName();
      nn.onControlTaskEnd.connect(
          boost::bind(&PolygonalConfigurableRobotShapeModel::onStep, this,
                      boost::bind(&NavigationNode::getBaseFrameName, _1)));
    }
};
}
}

#endif /* CONFIGURABLEPOLYGONALSHAPE_H_ */
