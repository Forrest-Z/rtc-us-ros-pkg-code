/*
 * default_clearance_cost_strategy.h
 *
 *  Created on: Oct 5, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef DEFAULT_CLEARANCE_COST_STRATEGY_H_
#define DEFAULT_CLEARANCE_COST_STRATEGY_H_

#include <rtcus_dwa/clearance_strategies/clearance_cost_strategy_base.h>
#include <rtcus_dwa/clearance_strategies/obstacle_collision_info.h>

namespace rtcus_dwa
{
using namespace std;
using namespace boost;

template<typename ObstacleInfoType>
  class ObjectInfoDefaultClerance : public ClearanceCostStrategyBase
  {
  protected:
    ObstacleInfoType collision_point_;
    std::vector<ObstacleInfoType> obstacle_collision_info_;
  public:

    virtual ~ObjectInfoDefaultClerance()
    {
    }

    const ObstacleInfoType& getObstacleCollision() const
    {
      return this->collision_point_;
    }

    virtual void computeClearanceAllObstacles(const DwaObstacleVector & obstacles)
    {
      if (this->obstacle_collision_info_.size() != obstacles.size())
        this->obstacle_collision_info_.resize(obstacles.size());

      for (unsigned long i = 0; i < obstacles.size(); i++)
      {
        ObstacleInfoType & obstacle_info = this->obstacle_collision_info_[i];
        obstacle_info.init();
        obstacle_info.coordinates = obstacles[i];
        this->onObstacleComputation(obstacle_info);
      }
      this->obstacles_processed_ = true;
      this->postObstacleInfoReduction(obstacle_collision_info_);
    }

    void postObstacleInfoReduction(std::vector<ObstacleInfoType>& obstacles)
    {
      for (unsigned long i = 0; i < obstacle_collision_info_.size(); i++)
      {
        const ObstacleInfoType& obstacle_info = this->obstacle_collision_info_[i];

        //SELECT NEAREST COLLISION
        if (obstacle_info.hasTrajectoryCollision()
            && (!this->current_cost_->collision
                || obstacle_info.getCollisionDistance() < this->collision_point_.getCollisionDistance()))
        {
          this->collision_point_ = obstacle_info;
          this->current_cost_->collision = true;
        }
      }
    }

    virtual void onObstacleComputation(ObstacleInfoType& obstacle_info) const
    {
      PointXY& obstacle = obstacle_info.coordinates;
      PointXY temp_nearest_collision;
      ClearanceCostInformation computed_dist;

      if (this->config_->use_clearance_lookup_table)
      {
        bool collision = this->lookup_config_.lookup_table_->computeClearance(this->current_action_->getAction(),
                                                                              obstacle, this->config_->getRobotShape(),
                                                                              computed_dist);
        if (collision)
        {
          temp_nearest_collision.x = temp_nearest_collision.y = 0;
          obstacle_info.setCollision(computed_dist.linear_collision_distance_, temp_nearest_collision);
          /*
           Twist2D action2;
           t_float expected_distance2;
           //CHECK IF MAPPING INDEX IS WELL DONE
           unsigned int expected_index = current_action_index_;
           this->get_action_from_index(expected_index, action2);
           bool collisin_1 = this->trajectory_clearance_->computeClearance(action2, obstacle, config_->getRobotShape(),
           expected_distance2);
           //CHECK LOOKUP TABLE WORKS WELL
           t_float expected_distance;
           bool collisin_2 = this->trajectory_clearance_->computeClearance(this->current_action_, obstacle,
           config_->getRobotShape(), expected_distance);

           if (collisin_1 && collisin_2)
           ROS_INFO(
           "[linear: %f, angular:%f] Lookup distance: %lf. Expected lookup distance: %lf... really expected distance: %lf", this->current_action_.linear, this->current_action_.angular, computed_dist, expected_distance2, expected_distance);
           else
           ROS_INFO(
           "[linear: %f, angular:%f] Lookup distance: %lf, Lookup detected collision. but others didn't detecte it", this->current_action_.linear, this->current_action_.angular, computed_dist);
           */
        }
        //---------------------------------------------------------------
      }
      else
      {
        t_float second_distance_dist;
        PointXY temp_distant_collision;

        shared_ptr<CircularTrajectoryClearanceForCircularRobot> circular_clearance = boost::dynamic_pointer_cast<
            CircularTrajectoryClearanceForCircularRobot>(this->trajectory_clearance_);

        RTCUS_ASSERT_MSG(
            circular_clearance,
            "By the moment this clearance strategy should be based on circular robot and circular trajectories");
        CircularTrajectoryClearanceForCircularRobot::ClearanceResult distanceResult =
            circular_clearance->computeClearance(this->current_action_->getAction(), obstacle,
                                                 this->config_->getRobotShape().getRadius(),
                                                 this->config_->max_collision_distance,
                                                 this->current_action_->getAction().linear >= 0.0,
                                                 computed_dist.linear_collision_distance_, second_distance_dist,
                                                 temp_nearest_collision, temp_distant_collision,
                                                 this->config_->obstacle_inflation);

        //Tag obstacle as the robot is inside it and the trajectory inside it
        if (distanceResult == CircularTrajectoryClearanceForCircularRobot::COLLISION)
        {
          //obstacle_info.setCollisionDistance(computed_dist);
          obstacle_info.setCollision(computed_dist.linear_collision_distance_, temp_nearest_collision);
          //search de minimum... for optimization
          circular_clearance->max_distance_ = computed_dist.linear_collision_distance_;

          /*
           t_float lookup_dist;
           bool col = this->trajectory_clearance_lookup_->computeClearance(this->current_action_, obstacle,
           config_->getRobotShape(), lookup_dist);
           ROS_INFO(
           "[linear: %f, angular:%f] [ox %lf oy %lf]Lookup[k_index %d] distance: %lf [collision %d]. Expected distance: %lf error (%lf)", this->current_action_.linear, this->current_action_.angular, obstacle.x, obstacle.y, this->current_action_index_, lookup_dist, col, computed_dist, lookup_dist/computed_dist);
           */
        }
      }
    }

    virtual void preComputeCommand()
    {
      ClearanceCostStrategyBase::preComputeCommand();
    }

  }
  ;

template<typename ObstacleInfoType>
  class DefaultClearanceCostStrategy : public ObjectInfoDefaultClerance<ObstacleInfoType>
  {
  public:
    DefaultClearanceCostStrategy();
    virtual ~DefaultClearanceCostStrategy();

  }
  ;

}

#endif /* DEFAULT_CLEARANCE_COST_STRATEGY_H_ */
