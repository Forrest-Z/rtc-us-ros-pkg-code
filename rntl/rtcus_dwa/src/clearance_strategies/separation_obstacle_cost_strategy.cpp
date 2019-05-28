/*
 * separation_obstacle_cost_strategy.cpp
 *
 *  Created on: Oct 8, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_dwa/clearance_strategies/separation_obstacle_cost_strategy.h>
#include <rtcus_dwa/common.h>
#include <rtcus_dwa/collision_distance_methods.h>
#include <rtcus_assert/rtcus_assert.h>
#include <boost/math/distributions/logistic.hpp>

namespace rtcus_dwa
{
using namespace std;

ObstacleSeparationClearanceCostStrategy::~ObstacleSeparationClearanceCostStrategy()
{
}

void ObstacleSeparationClearanceCostStrategy::preComputeCommand()
{
  this->has_repulsive_point_ = false;
}

bool ObstacleSeparationClearanceCostStrategy::hasRepulsivePoint() const
{
  return has_repulsive_point_;
}

const ObstacleCollisionInfo& ObstacleSeparationClearanceCostStrategy::getRepulsivePoint() const
{
  RTCUS_ASSERT(has_repulsive_point_);
  return repulsive_point_;
}

void ObstacleSeparationClearanceCostStrategy::onObstacleComputation(ObstacleCollisionInfo& obstacle_info)
{
  ObjectInfoDefaultClerance<ObstacleCollisionInfo>::onObstacleComputation(obstacle_info);
  const PointXY& o = obstacle_info.coordinates;

  if (!obstacle_info.hasTrajectoryCollision())
  {
    double linear_1, separation_1, linear_2, separation_2;
    PointXY repulsive_1, repulsive_2;
    proyectObstacleToCircularTrajectory(current_action_.linear, current_action_.angular, o.x, o.y, separation_1,
                                        linear_1, repulsive_1, separation_2, linear_2, repulsive_2);

    obstacle_info.setCollisionProyectionPoint(repulsive_1, separation_1, linear_1, 0);
    obstacle_info.setSecondaryCollisionProyectionPoint(repulsive_2, separation_2, linear_2);
  }
}

void ObstacleSeparationClearanceCostStrategy::setRepulsivePoint(const ObstacleCollisionInfo& repulsive_point_)
{
  this->repulsive_point_ = repulsive_point_;
  has_repulsive_point_ = true;
}

void ObstacleSeparationClearanceCostStrategy::postObstacleInfoReduction(std::vector<ObstacleCollisionInfo>& obstacles)
{
  ObjectInfoDefaultClerance::postObstacleInfoReduction(obstacles);
  double collision_distance = -1;
  if (this->hasTrajectoryCollision())
    collision_distance = this->getObstacleCollision().getCollisionDistance();

  for (unsigned long i = 0; i < obstacles.size(); i++)
  {
    ObstacleCollisionInfo& obstacle_info = obstacles[i];
    if (!obstacle_info.hasTrajectoryCollision())
    {
      double collision_distance =
          this->hasTrajectoryCollision() ?
              this->getObstacleCollision().getCollisionDistance() - this->config_->security_area_obstacle_inflation :
              -1;

      double linear_1 = obstacle_info.getLinearProyectedDistance();
      double separation_1 = obstacle_info.getSeparationDistance();
      const PointXY& repulsive_1 = obstacle_info.getProyectedCoordinates();
      double linear_2 = obstacle_info.getSecondaryLinearProyectedDistance();
      double separation_2 = obstacle_info.getSecondarySeparationDistance();
      const PointXY& repulsive_2 = obstacle_info.getSecondaryProyectedCoordinates();

      t_float p1_cost = this->getNormalRepulsionObstacleCost(separation_1, linear_1);
      t_float p2_cost = this->getNormalRepulsionObstacleCost(separation_2, linear_2);

      if (collision_distance == -1 || (linear_1 < collision_distance && linear_2 < collision_distance))
      {
        if (separation_1 < separation_2)
        {
          obstacle_info.setCollisionProyectionPoint(repulsive_1, separation_1, linear_1, p1_cost);
          obstacle_info.setSecondaryCollisionProyectionPoint(repulsive_2, separation_2, linear_2);
        }
        else
        {
          obstacle_info.setCollisionProyectionPoint(repulsive_2, separation_2, linear_2, p2_cost);
          obstacle_info.setSecondaryCollisionProyectionPoint(repulsive_1, separation_1, linear_1);
        }
      }
      else if (linear_1 < collision_distance)
      {
        obstacle_info.setCollisionProyectionPoint(repulsive_1, separation_1, linear_1, p1_cost);
        obstacle_info.setSecondaryCollisionProyectionPoint(repulsive_2, separation_2, linear_2);
      }
      else if (linear_2 < collision_distance)
      {
        obstacle_info.setCollisionProyectionPoint(repulsive_2, separation_2, linear_2, p2_cost);
        obstacle_info.setSecondaryCollisionProyectionPoint(repulsive_1, separation_1, linear_1);
      }
      else
      {
        RTCUS_ASSERT("this case is not possible");
      }
    }
  }

  double max_cost = -1;
  for (unsigned long i = 0; i < obstacles.size(); i++)
  {
    const ObstacleCollisionInfo& obstacle_info = obstacles[i];

    if (!this->hasTrajectoryCollision()
        || (obstacle_info.hasProyectedObstacle() && obstacle_info.getLinearProyectedDistance() < collision_distance))
    {
      double repulsive_normal_cost = obstacle_info.getNormalRepulsionCost();
      if (!this->hasRepulsivePoint())
      //get first suitable separation distance
      {
        this->setRepulsivePoint(obstacle_info);
        max_cost = repulsive_normal_cost;
        //  ROS_INFO(
        //      "(v %lf omega %lf obstacle %ld) First cached separation distance, collision_distance %lf linear distance %lf", this->current_action_.linear, this->current_action_.angular, i, collision_distance, (!obstacle_info.hasTrajectoryCollision())? obstacle_info.getLinearProyectedDistance():-1);
      }

      else if (repulsive_normal_cost > max_cost)
      // get smaller separation distances
      {
        max_cost = repulsive_normal_cost;
        //double separation_distance = obstacle_info.getSeparationDistance();
        //double minimal_separation_distance = this->getRepulsivePoint().getSeparationDistance();
        //  ROS_INFO(
        //      " |- (v %lf omega %lf obstacle %ld) Cached smaller separation distance %lf older %lf, collision_distance %lf linear distance %lf", this->current_action_.linear, this->current_action_.angular, i, separation_distance, minimal_separation_distance, collision_distance, (!obstacle_info.hasTrajectoryCollision())? obstacle_info.getLinearProyectedDistance():-1);
        this->setRepulsivePoint(obstacle_info);
      }
    }

//POSTCONDITIONS
    if (!this->hasTrajectoryCollision())
    {
      //NO COLLISION -> PROYECTION
      RTCUS_ASSERT_MSG(
          hasRepulsivePoint(),
          " Theoretically, inside danger area and no collision, separation got... %d", hasRepulsivePoint());
    }

    else if (this->hasRepulsivePoint())
    {
      RTCUS_ASSERT(
          this->getObstacleCollision().getCollisionDistance() > this->getRepulsivePoint().getLinearProyectedDistance());
    }

  }
}

t_float ObstacleSeparationPonderatedClearanceCostStrategy::getNormalRepulsionObstacleCost(float separation_distance,
                                                                                          float linear_distance) const
{
  boost::math::logistic logistic(this->config_->clearance_sigmoid_location, this->config_->clearance_sigmoid_scale);
  double normalization_more_distant_collision_distance = this->normalization_distance_collision_;

  double nearness_cost = boost::math::cdf(logistic,
                                          (1 - linear_distance / normalization_more_distant_collision_distance));

  return nearness_cost;
}

t_float ObstacleSeparationClearanceCostStrategy::getNormalRepulsionObstacleCost(float separation_distance,
                                                                                float linear_distance) const
{
  boost::math::logistic logistic(this->config_->clearance_sigmoid_location, this->config_->clearance_sigmoid_scale);
  double normalization_more_distant_collision_distance = this->normalization_distance_collision_;

  /*
   double r = -1;
   if (this->current_action_.angular > DWA_MAXIMUM_RADIOUS_TO_AVOID_NUMERICAL_INESTABILITIES)
   r = 5 * abs(this->config_->get_v_botom() / this->config_->get_max_angular_velocity());

   r = max(r, normalization_more_distant_collision_distance);
   */
  double separation_cost = 1.0
      - boost::math::cdf(logistic, separation_distance / normalization_more_distant_collision_distance);
  return separation_cost;
}

t_float ObstacleSeparationClearanceCostStrategy::computeCost(double dtt_secs)
{
  double final_cost = -1;
  if (this->config_->hasCollision())
  {
    final_cost = 1.0;
  }
  else if (this->hasRepulsivePoint())
  {
    RTCUS_ASSERT(
        !this->hasTrajectoryCollision() || this->getObstacleCollision().getCollisionDistance() > this->getRepulsivePoint().getLinearProyectedDistance())

    const ObstacleCollisionInfo & more_repulsive_obstacle = this->getRepulsivePoint();
    final_cost = more_repulsive_obstacle.getNormalRepulsionCost();
  }
  else if (this->hasTrajectoryCollision())
  {
    //BAD COMMAND, collision with this danger obstacle
    final_cost = 1.0;
    //ROS_INFO("has trajectory collision but no repulsive point");
  }
  else
    RTCUS_ASSERT(false);

  RTCUS_ASSERT(final_cost != -1);
  //ROS_INFO("[Clearance Cost] (v %lf omega %lf) %lf", this->current_action_.linear, this->current_action_.angular, final_cost);

  return final_cost;
}

void ObstacleSeparationClearanceCostStrategy::render() const
{
  ObstacleSeparationClearanceCostStrategy::render();

  if (this->hasRepulsivePoint())
  {
    visualization_msgs::Marker m;
    m.ns = "danger_obstacle_key_point";

    //First show the ignored point
    {
      m.scale.x = m.scale.y = m.scale.z = 0.03;

      if (this->hasTrajectoryCollision()
          && this->getRepulsivePoint().getSecondaryLinearProyectedDistance()
              > this->getObstacleCollision().getCollisionDistance())
        m.type = visualization_msgs::Marker::CUBE;
      else
        m.type = visualization_msgs::Marker::SPHERE;
      m.pose.position.z = 0.01;
      m.pose.orientation.w = 1.0;

      m.pose.position.x = getRepulsivePoint().getSecondaryProyectedCoordinates().x;
      m.pose.position.y = getRepulsivePoint().getSecondaryProyectedCoordinates().y;

      m.color.b = 0.0;
      m.color.g = 1.0;
      m.color.r = 1.0;
      m.color.a = 1.0;

      this->onVisualRepresentation(m);
    }

    const PointXY& normal_point_ = this->getRepulsivePoint().getProyectedCoordinates();
    {
      m.scale.x = m.scale.y = m.scale.z = 0.03;
      m.type = visualization_msgs::Marker::SPHERE;
      m.pose.position.z = 0.01;
      m.pose.orientation.w = 1.0;

      m.pose.position.x = normal_point_.x;
      m.pose.position.y = normal_point_.y;

      m.color.b = 1.0;
      m.color.g = 0.0;
      m.color.r = 1.0;
      m.color.a = 1.0;

      this->onVisualRepresentation(m);
    }

    geometry_msgs::Point repulsive_point_msg;
    repulsive_point_msg.x = this->getRepulsivePoint().coordinates.x;
    repulsive_point_msg.y = this->getRepulsivePoint().coordinates.y;
    repulsive_point_msg.z = 0;

    geometry_msgs::Point repulsive_obstacle_msg;
    repulsive_obstacle_msg.x = normal_point_.x;
    repulsive_obstacle_msg.y = normal_point_.y;
    repulsive_obstacle_msg.z = 0;

    //B (from repulsive point to obstacle)
    {
      m.pose.position.x = 0;
      m.pose.position.y = 0;
      m.pose.position.z = 0;
      m.color.b = 1.0;
      m.color.g = 0.0;
      m.color.r = 1.0;
      m.color.a = 1.0;
      m.scale.x = m.scale.y = m.scale.z = 0.004;
      m.points.clear();

      m.type = visualization_msgs::Marker::LINE_LIST;
      m.points.push_back(repulsive_point_msg);
      m.points.push_back(repulsive_obstacle_msg);
      this->onVisualRepresentation(m);
    }

    //A (from ICR TO NEARESS)
    if (fabs(this->current_action_.angular) > DWA_MAXIMUM_RADIOUS_TO_AVOID_NUMERICAL_INESTABILITIES)
    {
      geometry_msgs::Point icr;
      {
        double radio = this->current_action_.linear / this->current_action_.angular;
        double icr_y = radio;

        m.type = visualization_msgs::Marker::SPHERE;
        m.points.clear();
        icr.x = 0;
        icr.y = icr_y;
        icr.z = 0;
        m.pose.position.x = 0;
        m.pose.position.y = icr_y;
        m.color.r = 0.0;
        this->onVisualRepresentation(m);

      }
      m.pose.position.x = 0;
      m.pose.position.y = 0;
      m.pose.position.z = 0;
      m.color.b = 1.0;
      m.color.g = 0.0;
      m.color.r = 0.0;
      m.color.a = 1.0;

      m.scale.x = m.scale.y = m.scale.z = 0.004;
      m.points.clear();
      m.type = visualization_msgs::Marker::LINE_LIST;
      m.points.push_back(icr);
      if (pow(repulsive_point_msg.x - icr.x, 2) + pow(repulsive_point_msg.y - icr.y, 2)
          < pow(repulsive_obstacle_msg.x - icr.x, 2) + pow(repulsive_obstacle_msg.y - icr.y, 2))
        m.points.push_back(repulsive_point_msg);
      else
        m.points.push_back(repulsive_obstacle_msg);

      this->onVisualRepresentation(m);

    }
  }
}

}

