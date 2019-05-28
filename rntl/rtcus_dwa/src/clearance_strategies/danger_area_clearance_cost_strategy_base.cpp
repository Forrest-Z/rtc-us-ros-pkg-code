/*
 * danger_area_clearance_cost_strategy_base.cpp
 *
 *  Created on: Oct 9, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_dwa/clearance_strategies/danger_area_clearance_cost_base.h>
#include <rtcus_assert/rtcus_assert.h>

namespace rtcus_dwa
{

bool DangerAreaClearanceCostBase::isInsideDangerArea() const
{
  return inside_danger_area_;
}

bool DangerAreaClearanceCostBase::isLoopingInsideDangerArea() const
{
  return danger_area_internal_loop_;
}

void DangerAreaClearanceCostBase::preComputeCommand()
{
  inside_danger_area_ = false;
  danger_area_internal_loop_ = false;
}

void DangerAreaClearanceCostBase::onObstacleComputation(ObstacleCollisionInfo& obstacle_info)
{

  //first call to the base algorithm to compute basic distance results
  ObjectInfoDefaultClerance::onObstacleComputation(obstacle_info);
  const PointXY& o = obstacle_info.coordinates;

  this->current_trajectory_clearance_strategy_->compute_second_point_ = true;
  CircularTrajectoryClearanceForCircularRobot::ClearanceResult distanceResult =
      this->current_trajectory_clearance_strategy_->computeClearance(this->current_action_, o,
                                                    this->config_->security_area_obstacle_inflation,
                                                    this->config_->max_collision_distance,
                                                    this->current_action_.linear >= 0.0,
                                                    obstacle_info.danger_area_entering_distance,
                                                    obstacle_info.danger_area_leaving_distance,
                                                    obstacle_info.danger_area_entering_point,
                                                    obstacle_info.danger_area_leaving_point,
                                                    this->config_->obstacle_inflation);

  if (distanceResult == CircularTrajectoryClearanceForCircularRobot::INTERNAL_LOOP)
    obstacle_info.danger_area_internal_loop_ = true;

  if (o.x * o.x + o.y * o.y
      < this->config_->security_area_obstacle_inflation * this->config_->security_area_obstacle_inflation)
    obstacle_info.setIsDangerousObstacle();

  if (distanceResult == CircularTrajectoryClearanceForCircularRobot::INTERNAL_LOOP)
    RTCUS_ASSERT(obstacle_info.isDangerousObstacle());

}

void DangerAreaClearanceCostBase::postObstacleInfoReduction(std::vector<ObstacleCollisionInfo>& obstacles)
{
  ObjectInfoDefaultClerance::postObstacleInfoReduction(obstacles);
  //ROS_INFO(
  //    "Computing normal point start. inside_danger_area %d, hasRepulsivePoint %d, trajectory collision:%d ", inside_danger_area_, hasRepulsivePoint(), this->hasTrajectoryCollision());

  for (unsigned long i = 0; i < obstacles.size(); i++)
  {
    const ObstacleCollisionInfo& obstacle_info = obstacles[i];

    this->inside_danger_area_ |= obstacle_info.isDangerousObstacle();
    this->danger_area_internal_loop_ |= obstacle_info.danger_area_internal_loop_;
  }
}

void DangerAreaClearanceCostBase::render() const
{
  for (unsigned int i = 0; i < this->obstacle_collision_info_.size(); i++)
  {
    visualization_msgs::Marker obstacle;
    //TODO: generalize this tf frame
    obstacle.header.stamp = ros::Time();
    obstacle.type = visualization_msgs::Marker::CYLINDER;
    obstacle.ns = "obstacles";

    obstacle.pose.position.x = this->obstacle_collision_info_[i].coordinates.x;
    obstacle.pose.position.y = this->obstacle_collision_info_[i].coordinates.y;

    obstacle.pose.position.z = -0.02;
    obstacle.pose.orientation.w = 1.0;

    obstacle.scale.x = this->config_->security_area_obstacle_inflation * 2;
    obstacle.scale.y = this->config_->security_area_obstacle_inflation * 2;
    obstacle.scale.z = 0.005;

    obstacle.color.b = 0.3;
    obstacle.color.g = 0.5;
    obstacle.color.r = 0.6;
    obstacle.color.a = 1.0;
    obstacle.lifetime = ros::Duration(10.0);
    this->onVisualRepresentation(obstacle);

    obstacle.ns = "danger_area_area";
    obstacle.scale.x = this->config_->getRobotShape().getRadius() * 2;
    obstacle.scale.y = this->config_->getRobotShape().getRadius() * 2;
    obstacle.scale.z = 0.1;
    obstacle.color.b = 0.2;
    obstacle.color.g = 0.6;
    obstacle.color.r = 0.8;
    this->onVisualRepresentation(obstacle);
  }
}
}

