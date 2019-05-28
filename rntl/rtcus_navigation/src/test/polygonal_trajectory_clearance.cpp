/*
 * polygonal_trajectory_clearance.cpp
 *
 *  Created on: Feb 24, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/duration.h>

#include <mrpt/utils/CTicTac.h>
#include <rtcus_navigation/collision_checkers/collision_cheker_polygonal_robot.h>
#include <rtcus_navigation/trajectory_clearance/discrete_collision_shaped_robot_clearance.h>
#include <rtcus_robot_shapes/polygonal_robot.h>
#include <rtcus_nav_msgs/Pose2D.h>
#include <pcl/point_types.h>

using namespace rtcus_nav_msgs;
using namespace pcl;

extern rtcus_robot_shapes::PolygonalRobot create_robot_shape();

TEST(ClearanceTests, CSTrajectoryPolygonalClearanceTest)
{
  rtcus_navigation::trajectory_clearance::DiscreteCollisionShapedRobotClearance clearance_detector;
  rtcus_robot_shapes::PolygonalRobot robot_shape = create_robot_shape();

  std::vector<Pose2D> trajectory;
  std::vector<PointXY> obstacles;
  rtcus_navigation::trajectory_clearance::DefaultClearanceCost clearance_cost;
  Pose2D p;
  trajectory.push_back(p);

  //CHECK NO COLLISION
  bool collision = clearance_detector.computeClearance(trajectory, obstacles, robot_shape, clearance_cost);
  ASSERT_FALSE(collision);
  ASSERT_FALSE(clearance_cost.collision);
  ASSERT_EQ(clearance_cost.linear_collision_distance_, std::numeric_limits<double>::max());

  //add some very close obstacles
  pcl::PointXY o;
  o.x = 0.5;
  o.y = 1.1;
  obstacles.push_back(o);

  o.x = 0.5;
  o.y = -0.1;
  obstacles.push_back(o);

  o.x = -0.1;
  o.y = 0.5;
  obstacles.push_back(o);

  collision = clearance_detector.computeClearance(trajectory, obstacles, robot_shape, clearance_cost);
  ASSERT_FALSE(collision);
  ASSERT_FALSE(clearance_cost.collision);
  ASSERT_EQ(clearance_cost.linear_collision_distance_, std::numeric_limits<double>::max());

  for (int i = 0; i < 10; i++)
  {
    p.x = i;
    trajectory.push_back(p);
  }

  o.x = 9;
  obstacles.push_back(o);

  collision = clearance_detector.computeClearance(trajectory, obstacles, robot_shape, clearance_cost);

  ASSERT_TRUE(collision);
  ASSERT_TRUE(clearance_cost.collision);
  ASSERT_LT(clearance_cost.linear_collision_distance_, 9);
  ASSERT_GT(clearance_cost.linear_collision_distance_, 7);

  o.x = 5;
  obstacles.push_back(o);
  collision = clearance_detector.computeClearance(trajectory, obstacles, robot_shape, clearance_cost);
  ASSERT_TRUE(collision);
  ASSERT_TRUE(clearance_cost.collision);
  ASSERT_LT(clearance_cost.linear_collision_distance_, 5);
  ASSERT_GT(clearance_cost.linear_collision_distance_, 3);

}
