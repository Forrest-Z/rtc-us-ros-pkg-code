/*
 * clearance_cost_strategy.cpp
 *
 *  Created on: Oct 7, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */
#include <rtcus_dwa/clearance_strategies/clearance_cost_strategy_base.h>
#include <boost/math/distributions/logistic.hpp>
#include <rtcus_assert/rtcus_assert.h>
#include<std_msgs/Float64.h>
#include <pcl/point_types.h>
#include <rtcus_nav_msgs/Twist2D.h>
#include <rtcus_navigation/trajectory_clearance/covariant_trajectory_clearance.h>

namespace rtcus_dwa
{
using namespace rtcus_navigation::trajectory_clearance;

void ClearanceCostStrategyBase::use_trajectory_rollout_circular_robot()
{
  typedef TrajectoryClearance<Twist2D, pcl::PointCloud<pcl::PointXY>, ICircularRobotShape, DefaultClearanceCost> TSrc;
  this->trajectory_clearance_ = ExplicitCovarianceTraits<TSrc, TClearanceStrategy>::CovariantDecorator(
      this->clearance_strategy_tr_circular_robot_);

  lookup_config_.setFromType(
      boost::static_pointer_cast<TrajectoryClearance<Twist2D, pcl::PointXY, ICircularRobotShape, DefaultClearanceCost> >(
          this->clearance_strategy_tr_circular_robot_));
}

void ClearanceCostStrategyBase::use_trajectory_rollout_and_shape_clearance_strategy()
{
  typedef TrajectoryClearance<Twist2D, pcl::PointCloud<pcl::PointXY>, IPolygonalRobotShape, DefaultClearanceCost> TSrc;
  this->trajectory_clearance_ = ExplicitCovarianceTraits<TSrc, TClearanceStrategy>::CovariantDecorator(
      this->clearance_strategy_tr_);

  lookup_config_.setFromType(
      boost::static_pointer_cast<TrajectoryClearance<Twist2D, pcl::PointXY, IPolygonalRobotShape, DefaultClearanceCost> >(
          clearance_strategy_tr_));

}

void ClearanceCostStrategyBase::use_circular_clearance_strategy()
{
  typedef TrajectoryClearance<Twist2D, pcl::PointCloud<pcl::PointXY>, ICircularRobotShape, DefaultClearanceCost> TSrc;
  this->trajectory_clearance_ = ExplicitCovarianceTraits<TSrc, TClearanceStrategy>::CovariantDecorator(
      this->clearance_strategy_circular_);

  lookup_config_.setFromType(
      boost::static_pointer_cast<TrajectoryClearance<Twist2D, pcl::PointXY, ICircularRobotShape, DefaultClearanceCost> >(
          clearance_strategy_circular_));
}

void ClearanceCostStrategyBase::onRobotShapeChange(const PolygonalRobot& shape, const DwaConfig& config)
{
  if (config.clerance_trajectory_generation == "CircularTrayectoriesCircularShape")
    use_circular_clearance_strategy();
  else if (config.clerance_trajectory_generation == "TrajectoryRolloutPolygonalShapeRobot")
    use_trajectory_rollout_and_shape_clearance_strategy();
  else if (config.clerance_trajectory_generation == "TrajectoryRolloutCircularShapeRobot")
    use_trajectory_rollout_circular_robot();
  else
    ROS_WARN(" DWA. Configuration updated but clearance trajectory generation method is unknown. Ignoring.");

  this->lookup_config_.precomputeLookupTableFromShape(shape, config);
}

//----------------------------------------------------------------------
ClearanceCostStrategyBase::ClearanceCostStrategyBase() :
    lookup_config_()
{
  ros::NodeHandle nh("~planner/trajectory_rollout_clearance");
  this->clearance_strategy_circular_ = boost::make_shared<CircularTrajectoryClearanceForCircularRobot>();
  this->clearance_strategy_tr_circular_robot_ = boost::make_shared<TrajectoryRolloutCircularRobot>();
  this->clearance_strategy_tr_circular_robot_->setTimeIntegrationPeriod(std::numeric_limits<double>::max()); //0.1 - now controlled by the precision and max Distance
  this->clearance_strategy_tr_circular_robot_->setTrajectoryPrecision(100);
  this->clearance_strategy_tr_circular_robot_->setMaxDistance(15);

  this->clearance_strategy_tr_ = boost::make_shared<TrajectoryRolloutPolygonalRobot>();
  this->clearance_strategy_tr_->setTimeIntegrationPeriod(std::numeric_limits<double>::max()); //0.1 - now controlled by the precision and max Distance
  this->clearance_strategy_tr_->setTrajectoryPrecision(100);
  this->clearance_strategy_tr_->setMaxDistance(15);

  double value;
  if (!nh.getParam("integration_period", value))
    nh.setParam("integration_period", this->clearance_strategy_tr_->getTimeIntegrationPeriod());
  else
  {
    this->clearance_strategy_tr_->setTimeIntegrationPeriod(value);
    this->clearance_strategy_tr_circular_robot_->setTimeIntegrationPeriod(value);
  }

  int value_int;
  if (!nh.getParam("trajectory_samples", value_int))
    nh.setParam("trajectory_samples", (int)this->clearance_strategy_tr_->getTrajectoryPrecision());
  else
  {
    this->clearance_strategy_tr_->setTrajectoryPrecision(value_int);
    this->clearance_strategy_tr_circular_robot_->setTrajectoryPrecision(value_int);
  }

  this->clearance_strategy_tr_->setStrategy(0);
  if (!nh.getParam("strategy", value_int))
    nh.setParam("strategy", this->clearance_strategy_tr_->getStrategy());
  else
  {
    this->clearance_strategy_tr_->setStrategy(value_int);
  }

  nh = ros::NodeHandle("~planner");
  std::string trajectory_generation;
  if (!nh.getParam("clearance_trajectory_generation", trajectory_generation)
      || trajectory_generation == "CircularTrayectoriesCircularShape")
  {
    this->use_circular_clearance_strategy();
  }
  else if (trajectory_generation == "TrajectoryRolloutCircularShapeRobot")
  {
    this->use_trajectory_rollout_circular_robot();
  }
  else if (trajectory_generation == "TrajectoryRolloutPolygonalShapeRobot")
  {
    this->use_trajectory_rollout_and_shape_clearance_strategy();
  }

  nh = ros::NodeHandle("~planner/dwa_clearance_info");
  dynamic_normalization_distance_pub_ = nh.advertise<std_msgs::Float64>("normalization_distance", 1);
  max_admisible_velocity_pub_ = nh.advertise<std_msgs::Float64>("max_admisible_velocity", 1);
}

ClearanceCostStrategyBase::~ClearanceCostStrategyBase()
{

}
//-------------------------------------------------------------------------
void ClearanceCostStrategyBase::init(const DwaConfig& config)
{
  this->config_ = &config;

  ROS_DEBUG("distance normalization");

  double v_ref = this->config_->get_v_top();
  this->worse_braking_distance = (v_ref * v_ref * this->config_->getKinodynamicConfig().linear_forward_speed_limit
      / (2 * this->config_->getKinodynamicConfig().linear_brake_limit));

  this->normalization_distance_collision_ = std::max(2.0 * worse_braking_distance,
                                                     this->config_->max_collision_distance);

  this->markers_init(worse_braking_distance);

//------------------------------------------------
  /* std_msgs::Float64 msg;
   msg.data = normalization_distance_collision_;
   dynamic_normalization_distance_pub_.publish(msg);
   msg.data = worse_braking_distance;
   max_admisible_velocity_pub_.publish(msg);
   */
//----------------------------------------------------
//normalization_distance_collision_ = min((float)this->config_->get_v_top(), worse_braking_distance);
//normalization_distance_collision_ = (float)this->config_->get_v_top();
//normalization_distance_collision_ = (float)this->config_->get_v_top();
//normalization_distance_collision_ *= this->config_->max_collision_distance;
//normalization_distance_collision_ = min((float)normalization_distance_collision_, (float)worse_braking_distance);
//float normalization_factor=config_->max_collision_distance;
//normalization_distance_collision_ = max(normalization_distance_collision_,
//                                       (double)this->config_->max_collision_distance);
//ROS_INFO("max admisible velocity in %lf %lf -> %lf", v, omega, worse_braking_distance);
}

void ClearanceCostStrategyBase::setCollisionChecker(boost::shared_ptr<CollisionCheker> collision_checker)
{
  this->collision_checker_ = collision_checker;
}

double ClearanceCostStrategyBase::getMaxDistance() const
{
  return numeric_limits<double>::quiet_NaN();
}

void ClearanceCostStrategyBase::setMaxDistance(double dist)
{
  RTCUS_ASSERT_MSG(false, "by the moment this parameter is not allowed to be changed by this method");
}

bool ClearanceCostStrategyBase::computeClearance(const CommandCost<Twist2D>& action,
                                                 const DwaObstacleVector & obstacles, const PolygonalRobot& robot_shape,
                                                 ClearanceCostInformation& cost) const
{
  return const_cast<ClearanceCostStrategyBase*>(this)->computeClearance(action, obstacles, robot_shape, cost);

}

bool ClearanceCostStrategyBase::computeClearance(const CommandCost<Twist2D>& action,
                                                 const DwaObstacleVector & obstacles, const PolygonalRobot& robot_shape,
                                                 ClearanceCostInformation& cost)
{
  this->current_action_ = &action;
  this->current_cost_ = &cost;
  this->preComputeCommand();
  this->computeClearanceAllObstacles(obstacles);
  this->obstacles_processed_ = true;
  cost.clearance_ = this->computeCost(this->config_->simulation_time_step);
  this->render();
  return cost.collision;
}

void ClearanceCostStrategyBase::computeClearanceAllObstacles(const DwaObstacleVector & obstacles)
{
  if (config_->use_clearance_lookup_table)
    this->current_cost_->collision = this->lookup_config_.lookup_table_->computeClearance(
        this->current_action_->getAction(), obstacles, config_->getRobotShape(), *this->current_cost_);

  else
  {
    this->current_cost_->collision = this->trajectory_clearance_->computeClearance(this->current_action_->getAction(),
                                                                                   obstacles, config_->getRobotShape(),
                                                                                   *this->current_cost_);

    if (this->config_->clerance_trajectory_generation == "TrajectoryRolloutCircularShapeRobot")
    {
      this->current_cost_->trajectory_ = &(this->clearance_strategy_tr_circular_robot_->getStateSpaceTrajectory());
    }
    else
      this->current_cost_->trajectory_ = NULL;
  }
}

void ClearanceCostStrategyBase::preComputeCommand()
{
  this->lookup_config_.update_and_compute_current_action_index(*this->current_action_);

  this->clearance_strategy_circular_->setMaxDistance(config_->max_collision_distance);
  this->clearance_strategy_tr_->setMaxDistance(config_->max_collision_distance);
  this->clearance_strategy_tr_circular_robot_->setMaxDistance(config_->max_collision_distance);
  this->clearance_strategy_circular_->obstacle_inflation_ = config_->obstacle_inflation;

  this->current_cost_->collision = false;
  this->current_cost_->linear_collision_distance_ = std::numeric_limits<t_float>::quiet_NaN();
  this->current_cost_->angular_collision_distance_ = std::numeric_limits<t_float>::quiet_NaN();
  this->obstacles_processed_ = false;
}

t_float ClearanceCostStrategyBase::computeCost(double dtt_secs)
{
  double normalization_more_distant_collision_distance = normalization_distance_collision_ * dtt_secs;
  boost::math::logistic logistic(this->config_->clearance_sigmoid_location, this->config_->clearance_sigmoid_scale);

  double final_cost = -1;
  if (this->config_->hasCollision())
  {
    final_cost = 1.0;
    //ROS_INFO("v %lf omega %lf MAX COST -> Collision", v, omega);
  }
  else if (this->current_cost_->collision)
  {
    double collision_distance = this->current_cost_->linear_collision_distance_;
    //if (collision_distance > normalization_more_distant_collision_distance)
    //  final_cost = 0.0;
    //else
    final_cost = (1.0 - boost::math::cdf(logistic, collision_distance / normalization_more_distant_collision_distance));

  }
  else
    final_cost = 0.0;

  return final_cost;
}

double ClearanceCostStrategyBase::get_worse_braking_distance()
{
  return this->worse_braking_distance;
}

void ClearanceCostStrategyBase::markers_init(double worse_braking_distance)
{
  /*
   visualization_msgs::Marker m;
   m.type = visualization_msgs::Marker::CYLINDER;
   m.pose.position.x = 0.0;
   m.pose.position.y = 0.0;
   m.pose.position.z = 0.00;
   m.pose.orientation.w = 1.0;

   m.scale.x = worse_braking_distance / 2.0;
   m.scale.y = worse_braking_distance / 2.0;
   m.scale.z = 0.03;

   m.color.b = 0.1;
   m.color.g = 0.1;
   m.color.r = 1.0;
   m.color.a = 0.1;
   m.ns = "max_collision_area";
   this->onVisualRepresentation(m);
   */
}

void ClearanceCostStrategyBase::render() const
{
  /*
   if (this->current_cost_->collision)
   {
   visualization_msgs::Marker m;
   PointXY& p = current_cost_->collision_point_;

   m.type = visualization_msgs::Marker::SPHERE;
   m.pose.position.x = p.x;
   m.pose.position.y = p.y;
   m.pose.position.z = 0.01;
   m.pose.orientation.w = 1.0;

   m.scale.x = 0.03;
   m.scale.y = 0.03;
   m.scale.z = 0.03;

   m.color.b = 0.1;
   m.color.g = 0.1;
   m.color.r = 1.0;
   m.color.a = 1.0;
   m.ns = "collision";

   this->onVisualRepresentation(m);
   }
   */
}
}

