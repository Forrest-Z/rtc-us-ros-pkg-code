/*
 * ArbitraryTrajectoryClearanceCircularRobot.h
 *
 *  Created on: Feb 19, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_navigation/trajectory_clearance/trajectory_rollout_clearance_circular_robot.h>
#include <rtcus_conversions/conversions.h>
#include <rtcus_assert/rtcus_assert.h>
#include <boost/foreach.hpp>
#include <rtcus_navigation/common.h>

namespace rtcus_navigation
{
namespace trajectory_clearance
{
using namespace rtcus_nav_msgs;
using namespace rtcus_robot_shapes::interfaces;
using namespace rtcus_navigation::kinodynamic_models;
using namespace rtcus_motion_models;

TrajectoryRolloutCircularRobot::TrajectoryRolloutCircularRobot() :
    state_space_trajectory_(20), cspace_trajectory_(20), max_simulation_time_(10.0)
{
  DefaultNonHolonomicKinodynamicModel::TKinoDynamicDescription desc;
  //maximum acceleration -> inmmediate velocity saturation -> circular trajectories
  desc.linear_acceleration_limit = numeric_limits<double>::max();
  desc.linear_brake_limit = numeric_limits<double>::max();
  desc.angular_acceleration_limit = numeric_limits<double>::max();
  desc.linear_forward_speed_limit = numeric_limits<double>::max();
  desc.angular_speed_limit = numeric_limits<double>::max();

  this->kinodynamic_model_ = boost::make_shared<DefaultNonHolonomicKinodynamicModel>();
  this->kinodynamic_model_->setKinoDynamics(desc);
  this->kinodynamic_model_->onKinodynamicsChanged.connect(
      bind(&TrajectoryRolloutCircularRobot::on_kinodinamic_changed, this, _1));
  //this->motion_model_->setKinodynamics(desc);
//----------------------------------------------------------------------------
  this->motion_model_ = shared_ptr<TrajectoryRolloutNonHolonomic2D>(new TrajectoryRolloutNonHolonomic2D(1.0, desc));
  this->setTimeIntegrationPeriod(std::numeric_limits<double>::max()); //orchestrated by the numer of samples
//------------------------------------------------------------------
  this->current_state_.pose.x = 0;
  this->current_state_.pose.y = 0;
  this->current_state_.pose.phi = 0;

  this->current_state_.twist.linear = 0;
  this->current_state_.twist.angular = 0;
  this->current_state_.twist.lateral = 0;

}
TrajectoryRolloutCircularRobot::~TrajectoryRolloutCircularRobot()
{
}
const std::vector<Pose2D>& TrajectoryRolloutCircularRobot::getConfigurationSpaceTrajectory() const
{
  return this->cspace_trajectory_;
}

const std::vector<DynamicState2D>& TrajectoryRolloutCircularRobot::getStateSpaceTrajectory() const
{
  return this->state_space_trajectory_;
}

void TrajectoryRolloutCircularRobot::setCurrentState(const rtcus_nav_msgs::DynamicState2D& current_state)
{
  this->current_state_ = current_state;
}

bool TrajectoryRolloutCircularRobot::computeClearance(const Twist2D& action,
                                                      const pcl::PointCloud<pcl::PointXY>& obstacles,
                                                      const ICircularRobotShape& shape,
                                                      DefaultClearanceCost& clearance) const
{
  return this->computeClearance_aux(action, obstacles, shape, clearance);
}

bool TrajectoryRolloutCircularRobot::computeClearance(const Twist2D& action, const std::vector<pcl::PointXY>& obstacles,
                                                      const ICircularRobotShape& shape,
                                                      DefaultClearanceCost& clearance) const
{
  return this->computeClearance_aux(action, obstacles, shape, clearance);
}
bool TrajectoryRolloutCircularRobot::computeClearance(const Twist2D& action, const pcl::PointXY& obstacle,
                                                      const ICircularRobotShape& shape,
                                                      DefaultClearanceCost& clearance) const
{
  std::vector<pcl::PointXY> obstacles(1);
  obstacles[0] = obstacle;
  return this->computeClearance_aux(action, obstacles, shape, clearance);
}

template<typename TObstacles>
  bool TrajectoryRolloutCircularRobot::computeClearance_aux(const Twist2D& action, const TObstacles& obstacles,
                                                            const ICircularRobotShape& shape,
                                                            DefaultClearanceCost& clearance) const
  {
    DefaultNonHolonomicKinodynamicModel::TKinoDynamicDescription desc;
    this->kinodynamic_model_->getKinodynamics(desc);
    //printf("linear velocity: %lf\n", desc.linear_forward_speed_limit);
    //printf("max distance: %lf\n", this->getMaxDistance());

    //double time_to_max = this->getMaxDistance() / desc.linear_forward_speed_limit;
    //printf("time_to_max %lf\n", time_to_max);

    //printf("At %lf \n", this->motion_model_->getTimeIntegrationPeriod());
    double simulation_time = std::min(this->max_simulation_time_.toSec(),
                                      this->getMaxDistance() / (std::max(fabs(action.linear), fabs(action.angular))));

    ros::Duration simulation_duration = ros::Duration(simulation_time);
    this->motion_model_->sampleStates(this->current_state_, action, simulation_duration,
                                      const_cast<TrajectoryRolloutCircularRobot*>(this)->state_space_trajectory_);

    //TODO: optimize and avoid copy, do an interator strategy move to rtcus_convert
    vector<Pose2D>& cspace_trajectory = const_cast<TrajectoryRolloutCircularRobot&>(*this).cspace_trajectory_;
    int i = 0;
    BOOST_FOREACH(const DynamicState2D& x, this->state_space_trajectory_)
    {
      cspace_trajectory[i++] = x.pose;
      //std::stringstream ss;
      //ss << x;
      //ROS_INFO("STATE \n:%s\n ", ss.str().c_str());
    }
    return this->internal_trajectory_clearance_.computeClearance(cspace_trajectory_, obstacles, shape, clearance);
  }

NonHolonomicKinodynamicModel& TrajectoryRolloutCircularRobot::getKinodynamicModel()
{
  return *this->kinodynamic_model_;
}

void TrajectoryRolloutCircularRobot::setMaxDistance(double distance)
{
  this->internal_trajectory_clearance_.setMaxDistance(distance);
}

void TrajectoryRolloutCircularRobot::setTrajectoryPrecision(unsigned int samples)
{
  ROS_INFO(" (%s). Setting trajectory space precision to %d", getClassName(*this).c_str(), samples);
  this->state_space_trajectory_.reserve(samples);
  this->state_space_trajectory_.resize(samples);
  this->cspace_trajectory_.reserve(samples);
  this->cspace_trajectory_.resize(samples);
}

unsigned int TrajectoryRolloutCircularRobot::getTrajectoryPrecision() const
{
  return this->state_space_trajectory_.size();
}

double TrajectoryRolloutCircularRobot::getMaxDistance() const
{
  return this->internal_trajectory_clearance_.getMaxDistance();
}

void TrajectoryRolloutCircularRobot::on_kinodinamic_changed(
    const NonHolonomicKinodynamicModel::TKinoDynamicDescription& config)
{
  this->motion_model_->setKinodynamics(config);
}

void TrajectoryRolloutCircularRobot::setTimeIntegrationPeriod(double period)
{
  printf("\n * Numerical integration period for the trajectory rollout algorithm set to %lf", period);
  double simulation_time = 10;
  printf("\n * for an example of simulation time of %lf secs: ", simulation_time);
  printf("\n * (%s) simulation time for this trajectory: %lf", getClassName(*this).c_str(), simulation_time);
  printf("\n * (%s) integration time for this trajectory: %lf", getClassName(*this).c_str(),
         this->motion_model_->getTimeIntegrationPeriod());
  printf("\n * (%s) expected number of loops %lf for %ld states sampled", getClassName(*this).c_str(),
         simulation_time / this->motion_model_->getTimeIntegrationPeriod(), this->state_space_trajectory_.size());

  this->motion_model_->setTimeIntegrationPeriod(period);
}
double TrajectoryRolloutCircularRobot::getTimeIntegrationPeriod() const
{
  return this->motion_model_->getTimeIntegrationPeriod();
}
}
}

