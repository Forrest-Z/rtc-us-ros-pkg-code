/*
 * trajectory_rollout_polygonal_robot.h
 *
 *  Created on: Feb 24, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef TRAJECTORY_ROLLOUT_POLYGONAL_ROBOT_H_
#define TRAJECTORY_ROLLOUT_POLYGONAL_ROBOT_H_

#include <rtcus_navigation/trajectory_clearance/trajectory_clearance.h>
#include <rtcus_navigation/trajectory_clearance/discrete_collision_shaped_robot_clearance.h>
#include <rtcus_nav_msgs/Twist2D.h>
#include <rtcus_robot_shapes/interfaces/polygonal.h>
#include <rtcus_navigation/kinodynamic_models/non_holonomic_kinodynamics.h>
#include <rtcus_motion_models/motion_models/non_holonomic_trajectory_rollout_2d.h>
#include <rtcus_nav_msgs/DynamicState2D.h>
#include <rtcus_navigation/trajectory_clearance/default_clearance_cost.h>

//obstacles
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>

namespace rtcus_navigation
{
namespace trajectory_clearance
{
using namespace rtcus_nav_msgs;
using namespace rtcus_robot_shapes::interfaces;
using namespace boost;
using namespace rtcus_navigation::kinodynamic_models;
using namespace std;

class TrajectoryRolloutPolygonalRobot : public TrajectoryClearance<Twist2D, pcl::PointCloud<pcl::PointXY>,
                                            IPolygonalRobotShape, DefaultClearanceCost>,
                                        public TrajectoryClearance<Twist2D, std::vector<pcl::PointXY>,
                                            IPolygonalRobotShape, DefaultClearanceCost>,
                                        public TrajectoryClearance<Twist2D, pcl::PointXY, IPolygonalRobotShape,
                                            DefaultClearanceCost>
{
protected:
  DiscreteCollisionShapedRobotClearance internal_cspace_trajectory_clearance_;
  DynamicState2D current_state_;
  shared_ptr<NonHolonomicKinodynamicModel> kinodynamic_model_;
  shared_ptr<rtcus_motion_models::TrajectoryRolloutNonHolonomic2D> motion_model_;
  vector<DynamicState2D> state_space_trajectory_;
  vector<Pose2D> cspace_trajectory_;

public:
  TrajectoryRolloutPolygonalRobot();
  virtual ~TrajectoryRolloutPolygonalRobot();
  virtual void setCurrentState(const rtcus_nav_msgs::DynamicState2D& current_state);
  virtual bool computeClearance(const Twist2D& action, const pcl::PointCloud<pcl::PointXY>& obstacles,
                                const IPolygonalRobotShape& shape, DefaultClearanceCost& clearance) const;
  virtual bool computeClearance(const Twist2D& action, const std::vector<pcl::PointXY>& obstacles,
                                const IPolygonalRobotShape& shape, DefaultClearanceCost& clearance) const;
  virtual bool computeClearance(const Twist2D& action, const pcl::PointXY& obstacles, const IPolygonalRobotShape& shape,
                                DefaultClearanceCost& clearance) const;

  virtual const std::vector<Pose2D>& getTrajectory() const
  {
    return this->cspace_trajectory_;
  }

  NonHolonomicKinodynamicModel& getKinodynamicModel();

  /*\brief max distance in meters  . Can be useful both: for navigation customization and performance issues*/
  virtual void setMaxDistance(double distance);
  inline double getMaxDistance() const;

  //\brief number of samples of the trajectory
  virtual unsigned int getTrajectoryPrecision() const;
  virtual void setTrajectoryPrecision(unsigned int samples);

  //\brief precision between two sampled states
  virtual void setTimeIntegrationPeriod(double At);
  virtual double getTimeIntegrationPeriod() const;

  void setStrategy(int id)
  {
    this->internal_cspace_trajectory_clearance_.strategy = id;
  }
  double getStrategy() const
  {
    return this->internal_cspace_trajectory_clearance_.strategy;
  }

protected:
  void on_kinodinamic_changed(const NonHolonomicKinodynamicModel::TKinoDynamicDescription&);
  template<typename TObstacles>
    bool computeClearance_aux(const Twist2D& action, const TObstacles& obstacles, const IPolygonalRobotShape& shape,
                              DefaultClearanceCost& clearance) const;
};
}
}

#endif /* TRAJECTORY_ROLLOUT_POLYGONAL_ROBOT_H_ */
