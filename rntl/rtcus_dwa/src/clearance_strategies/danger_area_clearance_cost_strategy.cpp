/*
 * danger_area_clearance_cost_strategy.cpp
 *
 *  Created on: Oct 8, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_dwa/clearance_strategies/danger_area_clearance_cost_strategy.h>
#include <rtcus_assert/rtcus_assert.h>
#include <boost/math/distributions/logistic.hpp>

namespace rtcus_dwa
{

void DangerAreaClearanceCostStrategy::setLeavingDangerAreaPoint(double dist, const PointXY& p)
{
  RTCUS_ASSERT(dist >= 0);
  distance_leaving_danger_area_ = dist;
  leaving_point_ = p;
}

void DangerAreaClearanceCostStrategy::setEnteringDangerAreaPoint(double dist, const PointXY& p)
{
  RTCUS_ASSERT(dist >= 0);
  distance_entering_danger_area_ = dist;
  entering_point_ = p;
}

void DangerAreaClearanceCostStrategy::preComputeCommand()
{
  distance_entering_danger_area_ = -1;
  distance_leaving_danger_area_ = -1;
}

DangerAreaClearanceCostStrategy::~DangerAreaClearanceCostStrategy()
{
}

bool DangerAreaClearanceCostStrategy::accesibleLeavingAttractivePoint() const
{
  return distance_leaving_danger_area_ != -1;
}

const PointXY& DangerAreaClearanceCostStrategy::leavingAtractivePoint() const
{
  RTCUS_ASSERT(accesibleLeavingAttractivePoint());
  return leaving_point_;
}

double DangerAreaClearanceCostStrategy::leavingDistance() const
{
  RTCUS_ASSERT(accesibleLeavingAttractivePoint());
  return this->distance_leaving_danger_area_;
}

bool DangerAreaClearanceCostStrategy::accesibleEnteringRepulsivePoint() const
{
  return distance_entering_danger_area_ != -1;
}

double DangerAreaClearanceCostStrategy::enteringDistance() const
{
  RTCUS_ASSERT(accesibleEnteringRepulsivePoint());
  return this->distance_entering_danger_area_;
}

const PointXY& DangerAreaClearanceCostStrategy::enteringRepulsivePoint() const
{
  RTCUS_ASSERT(accesibleEnteringRepulsivePoint());
  return entering_point_;
}

double DangerAreaClearanceCostStrategy::normalize()
{
  double normalization_more_distant_collision_distance = this->normalization_distance_collision_;

  //--- NORMALIZE DANGER ENTERING AREA DISTANCE
  if (this->accesibleEnteringRepulsivePoint()
      && this->distance_entering_danger_area_ > normalization_more_distant_collision_distance)
    this->distance_entering_danger_area_ = normalization_more_distant_collision_distance;

  if (this->accesibleLeavingAttractivePoint()
      && this->distance_leaving_danger_area_ > normalization_more_distant_collision_distance)
    this->distance_leaving_danger_area_ = normalization_more_distant_collision_distance;

  return normalization_more_distant_collision_distance;
}

void DangerAreaClearanceCostStrategy::postObstacleInfoReduction(std::vector<ObstacleCollisionInfo>& obstacles)
{
  DangerAreaClearanceCostBase::postObstacleInfoReduction(obstacles);
  ROS_DEBUG("Computing attractive and repulsive points");
  //----------------------- EVALUATING DANGER AREA--------------------------------
  for (unsigned long i = 0; i < obstacles.size(); i++)
  {
    const ObstacleCollisionInfo& obstacle_info = obstacles[i];
    double mark_collision_distance = this->config_->max_collision_distance;

    if (this->hasTrajectoryCollision())
      mark_collision_distance = this->getObstacleCollision().getCollisionDistance();

    if (this->isInsideDangerArea())
    //cases C or D or F
    {

      if (!this->isLoopingInsideDangerArea())
      {
        //ROS_INFO("danger area of this obstacle");
        //take the nearest which verifies outside and nearest (repulsive) and before the collision
        if (!obstacle_info.isDangerousObstacle())
        {
          if (obstacle_info.danger_area_entering_distance < mark_collision_distance
              && (!this->accesibleEnteringRepulsivePoint()
                  || obstacle_info.danger_area_entering_distance < this->enteringDistance()))
          {
            this->setEnteringDangerAreaPoint(obstacle_info.danger_area_entering_distance,
                                             obstacle_info.danger_area_entering_point);
            ROS_DEBUG(
                "repulsive enter inside near distance: %lf - point (%lf %lf)", obstacle_info.danger_area_entering_distance, obstacle_info.danger_area_entering_point.x, obstacle_info.danger_area_entering_point.y);
          }
        }

        //take the more distant (of the leaving- nearest point and inside)  (atractive) and before the collision
        if (obstacle_info.isDangerousObstacle() && obstacle_info.danger_area_entering_distance < mark_collision_distance
            && (!this->accesibleLeavingAttractivePoint()
                || obstacle_info.danger_area_entering_distance > this->leavingDistance()))
        {
          bool far_obstacles = true;
          for (unsigned long j = 0; j < obstacles.size() && far_obstacles; j++)
          {
            PointXY & obstacle = obstacles[j].coordinates;
            double diffx = obstacle.x - obstacle_info.danger_area_entering_point.x;
            double diffy = obstacle.y - obstacle_info.danger_area_entering_point.y;

            double linear_distance_to_attractive_point = sqrt(diffx * diffx + diffy * diffy);
            if (linear_distance_to_attractive_point
                < this->config_->getRobotShape().getRadius() + this->config_->security_area_obstacle_inflation * 0.2)
              far_obstacles = false;
          }

          if (far_obstacles)
          {
            ROS_DEBUG(
                "atractive inside - near distance: %lf - point (%lf %lf)", obstacle_info.danger_area_entering_distance, obstacle_info.danger_area_entering_point.x, obstacle_info.danger_area_entering_point.y);
            this->setLeavingDangerAreaPoint(obstacle_info.danger_area_entering_distance,
                                            obstacle_info.danger_area_entering_point);
          }
        }
      }
    }
    else //cases A or (B or E)
    {
      //ROS_INFO("outside of this obstacle:");
      //take the nearest (repulsive)
      if (obstacle_info.danger_area_entering_distance < mark_collision_distance
          && (!this->accesibleEnteringRepulsivePoint()
              || obstacle_info.danger_area_entering_distance < this->enteringDistance()))
        this->setEnteringDangerAreaPoint(obstacle_info.danger_area_entering_distance,
                                         obstacle_info.danger_area_entering_point);

      //take the nearest possible and before the collision (atractive)
      /* if (obstacle_info.danger_area_leaving_distance < mark_collision_distance
       && (!clearance_.accesibleLeavingAttractivePoint()
       || obstacle_info.danger_area_leaving_distance < clearance_.leavingDistance()))
       {

       clearance_.setLeavingDangerAreaPoint(obstacle_info.danger_area_leaving_distance, obstacle_info.danger_area_leaving_point);
       }*/
      //if found case A
      //if not found cases B and E
    }
  }

  //BOOST_ASSERT(
  //    !this->isLoopingInsideDangerArea || (this->isLoopingInsideDangerArea && !this->accesibleLeavingAttractivePoint() && !this->accesibleEnteringRepulsivePoint()));
  ROS_DEBUG("Done computing attractive and repulsive points");

}

t_float DangerAreaClearanceCostStrategy::computeCost(double dtt_secs)
{

  //--- COMPUTE WEIGHTS FOR DANGER AREA AND COLLISION
  assert(
      this->config_->security_area_clearance_weight_obsolete>=0.0 && this->config_->security_area_clearance_weight_obsolete<=1.0);
  float obstacle_collision_weight = 1.0 - this->config_->security_area_clearance_weight_obsolete;
  boost::math::logistic logistic(this->config_->clearance_sigmoid_location, this->config_->clearance_sigmoid_scale);
  //------------------------------------------------------------------------------------------

  double cummulated_cost = 0;
  double cumulated_weight = 0.0;
  ROS_DEBUG(
      "collision distance (%lf) enter_distance (%lf) leave distance(%lf) ", this->getObstacleCollision().getCollisionDistance(), this->distance_entering_danger_area_, this->distance_leaving_danger_area_);
  double normalization_more_distant_collision_distance = normalize();
  ROS_DEBUG(
      "norm (%lf) -> collision distance (%lf) enter_distance (%lf) leave distance(%lf) -> ", normalization_more_distant_collision_distance, this->getObstacleCollision().getCollisionDistance(), this->distance_entering_danger_area_, this->distance_leaving_danger_area_);
  double collision_cost = -1, enter_cost = -1, leave_cost = -1;

  if (this->hasTrajectoryCollision())
  {
    collision_cost = obstacle_collision_weight
        * (1.0
            - boost::math::cdf(
                logistic,
                this->getObstacleCollision().getCollisionDistance() / normalization_more_distant_collision_distance));

    cummulated_cost += collision_cost;

    cumulated_weight += obstacle_collision_weight;
    ROS_DEBUG("addinng collision cost %lf cumm %lf", collision_cost, cummulated_cost);
  }
  if (this->accesibleEnteringRepulsivePoint())
  {
    enter_cost = this->config_->security_area_clearance_weight_obsolete
        * boost::math::cdf(logistic, 1.0 - (this->enteringDistance() / normalization_more_distant_collision_distance));
    cummulated_cost += enter_cost;

    cumulated_weight += this->config_->security_area_clearance_weight_obsolete;
    ROS_DEBUG("addinng enter cost %lf cumm %lf", enter_cost, cummulated_cost);
  }
  if (this->accesibleLeavingAttractivePoint() && this->isInsideDangerArea())
  {
    //best the nearest
    leave_cost = (1
        - (this->config_->security_area_clearance_weight_obsolete
            * boost::math::cdf(logistic, this->leavingDistance() / normalization_more_distant_collision_distance)));
    cummulated_cost -= leave_cost;
    ROS_DEBUG("addinng leave cost %lf cumm %lf", leave_cost, cummulated_cost);
    cumulated_weight += this->config_->security_area_clearance_weight_obsolete;

  }

  if (cummulated_cost < 0.0)
    cummulated_cost = 0.0;

  double final_cost = -1;
  if (this->isLoopingInsideDangerArea())
  {
    //inside loop
    ROS_DEBUG("very bad command cost 1.0 -- inside loop");
    final_cost = 1.0;
  }
  else if (this->isInsideDangerArea() && this->hasTrajectoryCollision() && !this->accesibleLeavingAttractivePoint())
  {

    //very bad command
    ROS_DEBUG("very bad command cost 1.0 -- inside and not leaving");
    final_cost = 1.0;

  }
  else if (cumulated_weight == 0.0)
  {
    ROS_DEBUG("very good command. cost 0.0 -- no expected_collision");
    //very good command
    final_cost = 0.0;
  }

  if (final_cost == -1)
  {
    ROS_DEBUG(
        " usual command -> collision cost (%lf) enter cost (%lf) leave cost(%lf)", collision_cost, enter_cost, leave_cost);
    final_cost = cummulated_cost / cumulated_weight;
  }

  ROS_DEBUG("TOTAL COST(%lf) -- cummulated (%lf) norm (%lf)", final_cost, cummulated_cost, cumulated_weight);
  BOOST_ASSERT(final_cost>=0.0 && final_cost<=1.0);
  return final_cost;
}

void DangerAreaClearanceCostStrategy::render() const
{
  DangerAreaClearanceCostBase::render();
  visualization_msgs::Marker m;
  m.ns = "danger_enter";
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.type = visualization_msgs::Marker::SPHERE;
  m.pose.position.z = 0.01;
  m.pose.orientation.w = 1.0;

  if (this->accesibleEnteringRepulsivePoint())
  {
    m.pose.position.x = this->enteringRepulsivePoint().x;
    m.pose.position.y = this->enteringRepulsivePoint().y;
    m.color.b = 0.0;
    m.color.g = 0.8;
    m.color.r = 0.8;
    m.color.a = 1.0;
    this->onVisualRepresentation(m);
  }

  if (this->accesibleLeavingAttractivePoint())
  {
    m.pose.position.x = this->leavingAtractivePoint().x;
    m.pose.position.y = this->leavingAtractivePoint().y;
    m.color.b = 0.0;
    m.color.g = 1.0;
    m.color.r = 0.0;
    m.color.a = 1.0;
    this->onVisualRepresentation(m);
  }
}
}

