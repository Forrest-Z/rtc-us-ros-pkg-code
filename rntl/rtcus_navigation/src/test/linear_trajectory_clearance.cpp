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
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <rtcus_nav_msgs/DynamicState2D.h>
#include <pcl/point_types.h>

#include <rtcus_navigation/trajectory_clearance/linear_trajectory_clearance_circular_shape.h>
#include <rtcus_robot_shapes/circular_robot.h>
#include <rtcus_compositions/state_composer.h>

using namespace pcl;

void fillObstacles(std::vector<PointXY>& obstacles)
{
  //add some very close obstacles
  pcl::PointXY o;
  o.x = 0.5;
  o.y = 0;
  obstacles.push_back(o);

  o.x = 1;
  o.y = 1;
  obstacles.push_back(o);
}

TEST(ClearanceTests, LinearContinousTrajectoryClearance)
{
  rtcus_navigation::trajectory_clearance::TwoPointSegment action;
  std::vector<PointXY> obstacles;
  rtcus_robot_shapes::CircularRobot robot_shape;
  robot_shape.setRadius(0.1);
  fillObstacles(obstacles);
  rtcus_navigation::trajectory_clearance::LinearTrajectoryClearance clearance_method;

  rtcus_navigation::trajectory_clearance::DefaultClearanceCost clearance_cost;

  action.x0 = 0;
  action.y0 = 0;
  action.x1 = 0.5;
  action.y1 = 0;

  bool collision = clearance_method.computeClearance(action, obstacles, robot_shape, clearance_cost);
  ASSERT_TRUE(collision);
  ASSERT_TRUE(clearance_cost.collision);
  ASSERT_LE(clearance_cost.linear_collision_distance_, 0.6+robot_shape.getRadius());
  ASSERT_GE(clearance_cost.linear_collision_distance_, 0.4-robot_shape.getRadius());

  action.x0 = 0;
  action.y0 = 0;
  action.x1 = 1.0;
  action.y1 = 0;

  collision = clearance_method.computeClearance(action, obstacles, robot_shape, clearance_cost);

  ASSERT_TRUE(collision);
  ASSERT_TRUE(clearance_cost.collision);
  ASSERT_LE(clearance_cost.linear_collision_distance_, 0.6+robot_shape.getRadius());
  ASSERT_GE(clearance_cost.linear_collision_distance_, 0.4-robot_shape.getRadius());

  action.x0 = 0;
  action.y0 = 0;
  action.x1 = 0.0;
  action.y1 = 0;

  collision = clearance_method.computeClearance(action, obstacles, robot_shape, clearance_cost);
  ASSERT_FALSE(collision);
  ASSERT_FALSE(clearance_cost.collision);

  action.x0 = 0;
  action.y0 = 0;
  action.x1 = 0.0;
  action.y1 = 1.0;

  collision = clearance_method.computeClearance(action, obstacles, robot_shape, clearance_cost);
  ASSERT_FALSE(collision);
  ASSERT_FALSE(clearance_cost.collision);

  action.x0 = 0;
  action.y0 = 0;
  action.x1 = 0.9;
  action.y1 = 0.9;

  collision = clearance_method.computeClearance(action, obstacles, robot_shape, clearance_cost);
  ASSERT_FALSE(collision);
  ASSERT_FALSE(clearance_cost.collision);

  action.x0 = 0;
  action.y0 = 0;
  action.x1 = 0.8;
  action.y1 = 0.8;

  collision = clearance_method.computeClearance(action, obstacles, robot_shape, clearance_cost);
  ASSERT_FALSE(collision);
  ASSERT_FALSE(clearance_cost.collision);

  action.x0 = 0;
  action.y0 = 0;
  action.x1 = 1.0;
  action.y1 = 1.0;

  collision = clearance_method.computeClearance(action, obstacles, robot_shape, clearance_cost);
  ASSERT_TRUE(collision);
  ASSERT_TRUE(clearance_cost.collision);
  ASSERT_LE(clearance_cost.linear_collision_distance_, sqrt(2)+robot_shape.getRadius());
  ASSERT_GE(clearance_cost.linear_collision_distance_, sqrt(2)-robot_shape.getRadius());

  action.x0 = 0;
  action.y0 = 0;
  action.x1 = 2.0;
  action.y1 = 2.0;

  collision = clearance_method.computeClearance(action, obstacles, robot_shape, clearance_cost);
  ASSERT_TRUE(collision);
  ASSERT_TRUE(clearance_cost.collision);
  ASSERT_LE(clearance_cost.linear_collision_distance_, sqrt(2)+robot_shape.getRadius());
  ASSERT_GE(clearance_cost.linear_collision_distance_, sqrt(2)-robot_shape.getRadius());
}

#include <rtcus_navigation/trajectory_clearance/trajectory_rollout_clearance_circular_robot.h>
#include <rtcus_nav_msgs/Twist2D.h>
using namespace geometry_msgs;
TEST(ClearanceTests, TrajectoryRolloutCircular)
{
  rtcus_navigation::trajectory_clearance::TrajectoryRolloutCircularRobot clearance_method;

  std::vector<pcl::PointXY> obstacles;
  fillObstacles(obstacles);

  rtcus_robot_shapes::CircularRobot robot_shape;
  robot_shape.setRadius(0.1);

  rtcus_nav_msgs::Twist2D action;
  action.linear = 1.0;
  action.angular = 0.0;

  rtcus_navigation::trajectory_clearance::DefaultClearanceCost clearance_cost;
  bool collision = clearance_method.computeClearance(action, obstacles, robot_shape, clearance_cost);
  ASSERT_TRUE(collision);
  ASSERT_TRUE(clearance_cost.collision);
  ASSERT_LE(clearance_cost.linear_collision_distance_, 0.6+robot_shape.getRadius());
  ASSERT_GE(clearance_cost.linear_collision_distance_, 0.4-robot_shape.getRadius());

  action.linear = 1.0;
  action.angular = 500.0;
  collision = clearance_method.computeClearance(action, obstacles, robot_shape, clearance_cost);
  ASSERT_FALSE(collision);
  ASSERT_FALSE(clearance_cost.collision);

  rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig kinodesc;
  clearance_method.getKinodynamicModel().getKinodynamics(kinodesc);
  kinodesc.angular_acceleration_limit = 0.001;
  clearance_method.getKinodynamicModel().setKinoDynamics(kinodesc);

  action.linear = 1.0;
  action.angular = 500.0;
  collision = clearance_method.computeClearance(action, obstacles, robot_shape, clearance_cost);
  ASSERT_TRUE(collision);
  ASSERT_TRUE(clearance_cost.collision);
  ASSERT_LE(clearance_cost.linear_collision_distance_, 0.6+robot_shape.getRadius());
  ASSERT_GE(clearance_cost.linear_collision_distance_, 0.4-robot_shape.getRadius());
}

#include <rtcus_navigation/trajectory_clearance/workspace_trajectory_2d_circular_robot_clearance.h>
#include <rtcus_nav_msgs/Pose2D.h>
using namespace geometry_msgs;
TEST(ClearanceTests, CircularWorkspace)
{
  std::vector<rtcus_nav_msgs::Pose2D> trajectory;
  std::vector<pcl::PointXY> obstacles;
  rtcus_robot_shapes::CircularRobot robot_shape;
  robot_shape.setRadius(0.1);
  fillObstacles(obstacles);
  rtcus_navigation::trajectory_clearance::WorkspaceTrajectory2DCircularRobotClearance clearance_detector;

  rtcus_navigation::trajectory_clearance::DefaultClearanceCost clearance_cost;
  rtcus_nav_msgs::Pose2D p;
  trajectory.push_back(p);

  p.x = 0.5;
  p.y = 0;
  trajectory.push_back(p);
  bool collision = clearance_detector.computeClearance(trajectory, obstacles, robot_shape, clearance_cost);
  ASSERT_TRUE(collision);
  ASSERT_TRUE(clearance_cost.collision);
  ASSERT_LE(clearance_cost.linear_collision_distance_, 0.6+robot_shape.getRadius());
  ASSERT_GE(clearance_cost.linear_collision_distance_, 0.4-robot_shape.getRadius());

  trajectory.clear();
  p.x = 0;
  p.y = 0;
  trajectory.push_back(p);
  p.x = 0.0;
  p.y = 1.0;
  trajectory.push_back(p);

  collision = clearance_detector.computeClearance(trajectory, obstacles, robot_shape, clearance_cost);
  ASSERT_FALSE(collision) << clearance_cost.linear_collision_distance_;
  ASSERT_FALSE(clearance_cost.collision);

  p.x = 1.0;
  p.y = 1.0;
  trajectory.push_back(p);
  collision = clearance_detector.computeClearance(trajectory, obstacles, robot_shape, clearance_cost);
  ASSERT_TRUE(collision) << clearance_cost.linear_collision_distance_;
  ASSERT_TRUE(clearance_cost.collision);
  ASSERT_LE(clearance_cost.linear_collision_distance_, 2.0+robot_shape.getRadius());
  ASSERT_GE(clearance_cost.linear_collision_distance_, 1.6-robot_shape.getRadius());
}
