/*
 * collision_checker_tests.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_navigation/collision_checkers/collision_cheker_polygonal_robot.h>
#include <rtcus_robot_shapes/polygonal_robot.h>
#include <gtest/gtest.h>

rtcus_robot_shapes::PolygonalRobot create_robot_shape()
{
  std::vector<pcl::PointXY> shape_points;
  pcl::PointXY p;

  p.x = 1.0;
  p.y = 0.0;
  shape_points.push_back(p);

  p.x = 1.0;
  p.y = 1.0;
  shape_points.push_back(p);

  p.x = 0.5;
  p.y = 0.5;
  shape_points.push_back(p);

  p.x = 0.0;
  p.y = 1.0;
  shape_points.push_back(p);

  p.y = 0.0;
  p.x = 0.0;
  shape_points.push_back(p);

  rtcus_robot_shapes::PolygonalRobot robot_shape;
  robot_shape.setPoints(shape_points);

  // ROBOT SHAPE
  //
  // 1.0_ ·       ·
  //      |\     /|
  // 0.5_ |  \ /  |
  //      |       |
  // 0.0_ |_______|
  //      |   |   |
  //      0  0.5  1
  return robot_shape;
}

TEST(CollisinonCheckers, BasicPolygonalCollisionCheckerTest)
{

  rtcus_navigation::collision_checkers::CollisionChekerPolygonalRobot collision_checker;
  rtcus_robot_shapes::PolygonalRobot robot_shape = create_robot_shape();

  pcl::PointXY p;
  std::vector<pcl::PointXY> obstacles;

  //outside obstacle
  p.x = 0.5;
  p.y = 1.0;
  obstacles.push_back(p);

  ASSERT_FALSE(collision_checker.detectCollision(obstacles,robot_shape));

  //inside obstacle
  p.x = 0.5;
  p.y = 0.25;
  obstacles.push_back(p);
  ASSERT_TRUE(collision_checker.detectCollision(obstacles,robot_shape));

}

