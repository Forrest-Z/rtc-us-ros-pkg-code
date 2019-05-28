/*
 * polygonal_robot.h
 *
 *  Created on: Dec 9, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef POLYGONAL_ROBOT_H_
#define POLYGONAL_ROBOT_H_

#include <rtcus_robot_shapes/interfaces/polygonal.h>
#include <rtcus_robot_shapes/interfaces/circular.h>
#include <rtcus_robot_shapes/PolygonalRobotShapeConfig.h>
#include <rtcus_robot_shapes/configurable_shape.h>
#include <geometry_msgs/Polygon.h>

namespace rtcus_robot_shapes
{

class PolygonalRobot : public interfaces::ICircularRobotShape, public interfaces::IPolygonalRobotShape,
                       public ConfigurableShape<PolygonalRobotShapeConfig>
{
protected:
  std::vector<pcl::PointXY> points_;
  std::vector<pcl::PointXY> triangle_buffer_;
  double maxium_radius_;

public:
  PolygonalRobot();
  PolygonalRobot(const PolygonalRobot&);
  PolygonalRobot(const PolygonalRobotShapeConfig&);
  PolygonalRobot& operator =(const PolygonalRobot&);
  virtual ~PolygonalRobot();

  virtual void clear();
  virtual void setPoints(const std::vector<pcl::PointXY>& points);

  virtual void getROSPolygon(geometry_msgs::Polygon&) const;
  virtual const std::vector<pcl::PointXY>& getPoints() const;
  inline virtual double getRadius() const
  {
    return maxium_radius_;
  }
  virtual void setRadius(double value);
  virtual const PolygonalRobotShapeConfig& setConfig(const PolygonalRobotShapeConfig& config);
  void getTriangularization(std::vector<pcl::PointXY>& triangle_buffer);

  friend std::ostream& operator<<(std::ostream &out, const PolygonalRobot&polygonal_shape);
};

}

#endif /* POLYGONAL_ROBOT_H_ */
