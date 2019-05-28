/*
 * TrajectoryRolloutRosLocalPlanner.cpp
 *
 *  Created on: Nov 16, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_trajectory_rollout_local_planner/trajectory_rollout_ros_local_planner.h>
#include <rtcus_navigation/world_perception.h>
#include <rtcus_navigation/navigation_node.h>

#include <base_local_planner/BaseLocalPlannerConfig.h>
#include <base_local_planner/costmap_model.h>
#include <navfn/navfn_ros.h>

#include <tf/transform_datatypes.h>
#include <rtcus_assert/rtcus_assert.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>

using namespace rtcus_navigation;
using namespace rtcus_nav_msgs;
using namespace boost;
using namespace base_local_planner;
using namespace geometry_msgs;
using namespace costmap_2d;

TrajectoryRolloutAlgorithmROS::TrajectoryRolloutAlgorithmROS() :
    path_(1)
{
  this->planner_ = boost::make_shared<base_local_planner::TrajectoryPlannerROS>();
  this->global_planner_ = boost::make_shared<navfn::NavfnROS>();
}
TrajectoryRolloutAlgorithmROS::~TrajectoryRolloutAlgorithmROS()
{
}

void TrajectoryRolloutAlgorithmROS::syntetizeTrajectory(const DynamicState2D& state, const Twist2D& action,
                                                        std::vector<PoseStamped>& path, std::string frame,
                                                        double sim_time)
{
  double x_i = state.pose.x;
  double y_i = state.pose.y;
  double theta_i = state.pose.phi;

  double vx_i, vtheta_i;

  vx_i = state.twist.linear;
  vtheta_i = state.twist.angular;

  long num_steps = path.size();

  //we at least want to take one step... even if we won't move, we want to score our current position
  if (num_steps == 0)
    num_steps = 1;

  path.clear();
  path.resize(num_steps);

  double dt = sim_time / num_steps;
  double time = 0.0;

  for (int i = 0; i < num_steps; ++i)
  {
    PoseStamped& p = path[i];
    p.pose.position.x = x_i;
    p.pose.position.y = y_i;
    p.pose.position.z = 0;
    p.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    p.header.frame_id = frame;

    //calculate velocities
    if ((action.linear - vx_i) >= 0)
      vx_i = std::min(action.linear, vx_i + v_forward_acceleration_ * dt);
    else
      vx_i = std::max(action.linear, vx_i - v_forward_acceleration_ * dt);

    //calculate velocities
    if ((action.angular - vtheta_i) >= 0)
      vtheta_i = std::min(action.angular, vtheta_i + angular_acceleration_ * dt);
    else
      vtheta_i = std::max(action.angular, vtheta_i - angular_acceleration_ * dt);

    //calculate positions
    x_i = x_i + (vx_i * cos(theta_i)) * dt;
    y_i = y_i + (vx_i * sin(theta_i)) * dt;
    ;
    theta_i = theta_i + vtheta_i * dt;

    //increment time
    time += dt;
  }

}
void TrajectoryRolloutAlgorithmROS::init(TNavigationNode& host_node)
{
  ROS_INFO("initializating the local planner %s", this->type_name().c_str());

  shared_ptr<rtcus_navigation::WorldPerceptionPort<Costmap2DROS> > costmap_perception_port = dynamic_pointer_cast<
      rtcus_navigation::WorldPerceptionPort<Costmap2DROS> >(host_node.getWorldPerceptionPort());

  ROS_INFO("Cheking if the costmap world perception model is being used");

  if (costmap_perception_port)
  {
    ROS_INFO("Got Costmap world perception model");

    //avoid the locking security and get the data pointer forever
    rtcus_stamp::Stamped<Costmap2DROS> cm;
    costmap_perception_port->getWorldEstimation(ros::Time::now(), cm);

    Costmap2DROS* costmap_controller = &(*cm);
    this->component_node_.param("local_planner/sim_time", sim_time_, 1.0);
    this->component_node_.param("local_planner/sim_granularity", sim_granularity_, 0.025);
    unsigned int num_steps = (unsigned int)(sim_time_ / sim_granularity_ + 0.5);
    this->path_.resize(num_steps);

    this->component_node_.param("local_planner/acc_lim_x", v_forward_acceleration_, 2.5);
    this->component_node_.param("local_planner/acc_lim_th", angular_acceleration_, 3.2);

    planner_->initialize(this->getArchitectureComponentName() + "/local_planner", &tf_, costmap_controller);
    this->global_planner_->initialize(this->getArchitectureComponentName() + "global_planner", costmap_controller);
  }
  else
  {
    ROS_INFO("NOT Got Costmap world perception model");
    RTCUS_ASSERT_MSG(false, "Incorrect World Perception Model");
  }
}
void TrajectoryRolloutAlgorithmROS::reset()
{

}

void TrajectoryRolloutAlgorithmROS::computeHolonomicDisktraPlan(const Costmap2DROS& obstacles, const pcl::PointXY& goal,
                                                                const DynamicState2D& state,
                                                                std::vector<PoseStamped>& path)
{
  PoseStamped pstart;
  pstart.header.frame_id = obstacles.getGlobalFrameID();
  pstart.header.stamp = ros::Time::now();

  rtcus_conversions::Conversions::convert(state.pose, pstart.pose);

  this->global_planner_->computePotential(pstart.pose.position);

  std::vector<PoseStamped> pre_path(this->sim_time_ * 3 / this->sim_granularity_);
  this->path_.clear();
  computeNonHolonomicSimplePlan(obstacles, goal, state, pre_path, this->sim_time_ * 3);
  ROS_INFO("Number of points of the simple goal trajectory %ld", pre_path.size());

  PoseStamped pend = pre_path.back();
  bool global_plan_computed = false;
  pend.header.frame_id = obstacles.getGlobalFrameID();
  pend.header.stamp = ros::Time::now();

  if (this->global_planner_->makePlan(pstart, pend, path))
  {
    global_plan_computed = true;
  }

  /*this->path_.clear();
   for (int i = pre_path.size() - 1 && !global_plan_computed; i >= 0; i--)
   {
   PoseStamped pend;
   PoseStamped& g = pre_path[i];

   ROS_INFO_STREAM("Checking point for goal "<< g);
   //check inside costmap
   pend.header.frame_id = obstacles.getGlobalFrameID();
   pend.header.stamp = ros::Time::now();
   rtcus_conversions::Conversions::convert(g, pend.pose);
   if (this->global_planner_->makePlan(pstart, pend, path))
   {
   global_plan_computed = true;
   ROS_INFO_STREAM("Global plan computed properly. Goal is:"<< g);
   break;
   }
   }

   if (!global_plan_computed)
   ROS_ERROR("Global plan could not be computed");*/
}

void TrajectoryRolloutAlgorithmROS::computeNonHolonomicSimplePlan(const Costmap2DROS& obstacles,
                                                                  const pcl::PointXY& goal, const DynamicState2D& state,
                                                                  std::vector<PoseStamped>& path, double sim_time)
{
  /*Getting geometrically the most direct trajectory to the goal*/
  double goaldist = sqrt(goal.x * goal.x + goal.y * goal.y);
  double middle = goaldist / 2.0;
  double alpha = atan(goal.y / goal.x);
  double omega = sin(alpha) / middle;

  rtcus_nav_msgs::Twist2D goal_action;
  goal_action.angular = omega;
  goal_action.linear = 1.0;
  goal_action.lateral = 0.0;

  syntetizeTrajectory(state, goal_action, path, obstacles.getGlobalFrameID(), sim_time);
}

bool TrajectoryRolloutAlgorithmROS::computeVelocityCommands(const Costmap2DROS& obstacles, const pcl::PointXY& goal,
                                                            const DynamicState2D& state, Twist2D& resulting_action)
{
  boost::mutex::scoped_lock l(m_mutex_);

  onPrecomputeCommand(*this);

  //computeNonHolonomicSimplePlan(obstacles, goal, state, this->path_,this->sim_time_);
  computeHolonomicDisktraPlan(obstacles, goal, state, this->path_);
  planner_->setPlan(path_);

  Twist cmd_res;
  bool ok = planner_->computeVelocityCommands(cmd_res);
  if (!ok)
    ROS_ERROR("The local plan was not properly computed");

  rtcus_conversions::Conversions::convert(cmd_res, resulting_action);
  this->onPostComputeCommand(*this, resulting_action, state, goal);
  return !ok;
}

