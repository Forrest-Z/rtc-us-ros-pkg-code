/*
 * clearance_cost_strategy.h
 *
 *  Created on: Sep 20, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef CLEARANCE_COST_STRATEGY_BASE_H_
#define CLEARANCE_COST_STRATEGY_BASE_H_

//subclearance strategies
#include <rtcus_navigation/trajectory_clearance/circular_trajectory_clearance_circular_robot.h>
#include <rtcus_navigation/trajectory_clearance/trajectory_rollout_clearance_circular_robot.h>
#include <rtcus_navigation/trajectory_clearance/trajectory_rollout_polygonal_robot.h>
#include <rtcus_dwa/clearance_strategies/lookup_table_proxy.h>

#include <rtcus_dwa/dwa_motion_model.h>
#include <rtcus_dwa/dwa_config.h>
#include <rtcus_dwa/collision_distance_methods.h>
#include <visualization_msgs/Marker.h>
#include <rtcus_dwa/dwa_command_cost.h>

namespace rtcus_dwa
{
using namespace rtcus_navigation::trajectory_clearance;
using namespace rtcus_robot_shapes::interfaces;
using namespace boost;

/**
 * The Polygonal robot shape allow us to use circular robot shapes or polygonal using trajectory clearance covariance
 * */
class ClearanceCostStrategyBase : public TrajectoryClearance<CommandCost<Twist2D>, DwaObstacleVector, PolygonalRobot,
    ClearanceCostInformation>
{
public:
  typedef TrajectoryClearance<Twist2D, pcl::PointCloud<pcl::PointXY>, PolygonalRobot, DefaultClearanceCost> TClearanceStrategy;

  ClearanceCostStrategyBase();
  virtual ~ClearanceCostStrategyBase();
  void setCollisionChecker(boost::shared_ptr<CollisionCheker> collision_checker);

  virtual void init(const DwaConfig& config);

  virtual bool computeClearance(const CommandCost<Twist2D>& action, const DwaObstacleVector& obstacles,
                                const PolygonalRobot& robot_shape, ClearanceCostInformation& clearance) const;

  virtual bool computeClearance(const CommandCost<Twist2D>& action, const DwaObstacleVector& obstacles,
                                const PolygonalRobot& robot_shape, ClearanceCostInformation& clearance);

  boost::signal<void(const visualization_msgs::Marker& markers)> onVisualRepresentation;
  virtual void onRobotShapeChange(const PolygonalRobot& shape, const DwaConfig& new_config);
  double get_worse_braking_distance();
  double worse_braking_distance;

  //----------------- QUERIES ---------------------------------------
  //bool hasTrajectoryCollision() const;
  //double getLinearCollisionDistance() const;
  //double collision_distance_;
  //bool has_collision_point_;

  virtual void setMaxDistance(double distance);
  virtual double getMaxDistance() const;

protected:
  //----------------- WORKFLOW ---------------------------------------
  virtual void preComputeCommand();
  void markers_init(double worse_braking_distance);
  virtual void render() const;
  virtual void computeClearanceAllObstacles(const DwaObstacleVector & obstacles);
  virtual t_float computeCost(double dtt_secs);

  double normalization_distance_collision_;
  ros::Publisher dynamic_normalization_distance_pub_;
  ros::Publisher max_admisible_velocity_pub_;

  //------------------- CLEARANCE STRATEGIES -----------------------------------------------------
protected:
  LookupConfig lookup_config_;

  void use_circular_clearance_strategy();
  void use_trajectory_rollout_and_shape_clearance_strategy();
  void use_trajectory_rollout_circular_robot();
  boost::shared_ptr<rtcus_dwa::CollisionCheker> collision_checker_;
  //---- current selected clearance strategy -----
  boost::shared_ptr<TClearanceStrategy> trajectory_clearance_;

  //---- clearance strategies choices -----
  boost::shared_ptr<TrajectoryRolloutPolygonalRobot> clearance_strategy_tr_;
  boost::shared_ptr<CircularTrajectoryClearanceForCircularRobot> clearance_strategy_circular_;
  boost::shared_ptr<TrajectoryRolloutCircularRobot> clearance_strategy_tr_circular_robot_;

  const DwaConfig* config_;
  const CommandCost<Twist2D>* current_action_;

  ClearanceCostInformation* current_cost_;

  bool obstacles_processed_;

private:

}
;
}

#endif /* CLEARANCE_COST_STRATEGY_H_ */

