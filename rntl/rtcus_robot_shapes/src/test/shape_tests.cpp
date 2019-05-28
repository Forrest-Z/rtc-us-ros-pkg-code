/*
 * shape_tests.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <ros/ros.h>
#include <rtcus_robot_shapes/polygonal_robot.h>

void test_triangularization()
{
  rtcus_robot_shapes::PolygonalRobot robot_shape;
  std::vector<pcl::PointXY> points;
  pcl::PointXY p;

  p.x = 1.0;
  p.y = 0.0;
  points.push_back(p);

  p.x = 1.0;
  p.y = 1.0;
  points.push_back(p);

  p.x=0.5;
  p.y=0.5;
  points.push_back(p);

  p.x = 0.0;
  p.y = 1.0;
  points.push_back(p);

  p.y = 0.0;
  p.x = 0.0;
  points.push_back(p);

  robot_shape.setPoints(points);
  std::vector<pcl::PointXY> triangle_buffer;
  robot_shape.getTriangularization(triangle_buffer);

  ROS_INFO("Total triangles %f", ((float)triangle_buffer.size()/3.0));
  for (int i = 0; i < triangle_buffer.size(); i += 3)
  {
    pcl::PointXY& p1 = triangle_buffer[i], &p2 = triangle_buffer[i + 1], &p3 = triangle_buffer[i + 2];
    ROS_INFO("Triangle: [%f,%f] [%f,%f] [%f,%f]", p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  test_triangularization();
}
