/*
 * trajectory_rollout_ros_local_planner.h
 *
 *  Created on: Nov 20, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef TRAJECTORY_ROLLOUT_ROS_LOCAL_PLANNER_H_
#define TRAJECTORY_ROLLOUT_ROS_LOCAL_PLANNER_H_

#include <rtcus_navigation/navigation_planner_base.h>
#include <rtcus_navigation/common.h>
#include <rtcus_navigation/core.h>
#include <rtcus_nav_msgs/Twist2D.h>
#include <rtcus_nav_msgs/DynamicState2D.h>

#include <base_local_planner/trajectory_planner_ros.h>
#include <navfn/navfn_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <rtcus_robot_shapes/polygonal_robot.h>
#include <rtcus_kinodynamic_description/NonHolonomicKinoDynamicsConfig.h>

class TrajectoryRolloutAlgorithmROS : public rtcus_navigation::NavigationPlanner<rtcus_nav_msgs::DynamicState2D,
    costmap_2d::Costmap2DROS, rtcus_nav_msgs::Twist2D, pcl::PointXY, rtcus_robot_shapes::PolygonalRobot,
    rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig, rtcus_navigation::ROSTimeModel>
{
public:
  DECLARE_NAVIGATION_PLANNER_TYPES_EXPLICIT(rtcus_nav_msgs::DynamicState2D, costmap_2d::Costmap2DROS, rtcus_nav_msgs::Twist2D, pcl::PointXY,rtcus_robot_shapes::PolygonalRobot,rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig,rtcus_navigation::ROSTimeModel);

private:
  boost::mutex m_mutex_;
  std::vector<geometry_msgs::PoseStamped> path_;

protected:
  boost::shared_ptr<base_local_planner::TrajectoryPlannerROS> planner_;
  boost::shared_ptr<navfn::NavfnROS> global_planner_;

  TNavigationNode* host_node;
  tf::TransformListener tf_;
  boost::shared_ptr<base_local_planner::WorldModel> world_model_;
  double sim_time_;
  double sim_granularity_;
  double v_forward_acceleration_;
  double angular_acceleration_;

  void computeNonHolonomicSimplePlan(const costmap_2d::Costmap2DROS& obstacles, const pcl::PointXY& goal,
                                     const rtcus_nav_msgs::DynamicState2D& state,
                                     std::vector<geometry_msgs::PoseStamped>& path, double sim_time);

  void computeHolonomicDisktraPlan(const costmap_2d::Costmap2DROS& obstacles, const pcl::PointXY& goal,
                                   const rtcus_nav_msgs::DynamicState2D& state,
                                   std::vector<geometry_msgs::PoseStamped>& path);

public:

  TrajectoryRolloutAlgorithmROS();
  virtual ~TrajectoryRolloutAlgorithmROS();
  virtual void init(TNavigationNode& host_node);
  virtual void reset();
  virtual bool computeVelocityCommands(const costmap_2d::Costmap2DROS& obstacles, const pcl::PointXY& goal,
                                       const rtcus_nav_msgs::DynamicState2D& state,
                                       rtcus_nav_msgs::Twist2D& resulting_action);
  //MotionModel
  virtual void syntetizeTrajectory(const rtcus_nav_msgs::DynamicState2D& state, const rtcus_nav_msgs::Twist2D& action,
                                   std::vector<geometry_msgs::PoseStamped>& path, std::string frame, double sim_time);
};

#endif /* TRAJECTORY_ROLLOUT_ROS_LOCAL_PLANNER_H_ */
