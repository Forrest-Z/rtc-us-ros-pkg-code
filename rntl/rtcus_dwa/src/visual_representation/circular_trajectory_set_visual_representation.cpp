/*
 * circular_trajectory_set_visual_representation.cpp
 *
 *  Created on: Nov 2, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_dwa/visual_representation/circular_trajectory_set_visual_representation.h>
#include <rtcus_nav_msgs/DynamicState2D.h>
#include <rtcus_conversions/conversions.h>
#include <rtcus_motion_models/motion_models/non_holonomic_2d.h>
#include <rtcus_dwa/dwa_config.h>
#include <rtcus_dwa/simple_dwa_ros.h>
#include <rtcus_navigation_tools/visual_representations/paint_trajectory.h>
#include <rtcus_nav_msgs/DynamicState.h>

namespace rtcus_dwa
{
namespace visual_representation
{
using namespace rtcus_navigation_tools;
using namespace std;
using namespace rtcus_nav_msgs;

template<typename GoalType, typename StateType>
  CircularTrajectorySetVisualRepresentation<GoalType, StateType>::~CircularTrajectorySetVisualRepresentation()
  {
  }

template<typename GoalType, typename StateType>
  CircularTrajectorySetVisualRepresentation<GoalType, StateType>::CircularTrajectorySetVisualRepresentation(
      const std::string& representation_frame, const std::string& local_frame, ros::NodeHandle& n) :
      VisualRepresetationMarkersBase(representation_frame, "dwa_uspace_trajectories", n), local_frame_(local_frame)
  {
    this->trajectory_ = boost::shared_ptr<vector<StateType> >(new vector<StateType>(50));
    last_trajectory_markers_count_ = 0;
  }

template<typename GoalType, typename StateType>
  void CircularTrajectorySetVisualRepresentation<GoalType, StateType>::updateCommand(
      const rtcus_nav_msgs::Twist2D& best_command, const StateType& local_state, const GoalType& local_goal)
  {
    this->best_command_ = best_command;
    this->local_goal_ = local_goal;
    this->local_state_ = local_state;
  }

void decorateTrajectory(visualization_msgs::Marker& line_strip, bool isAdmisible, float cost)
{
  line_strip.color.b = 0.1;
  line_strip.color.g = 0.1;
  line_strip.color.r = 0.1;
  line_strip.color.a = 1.0;

  if (!isAdmisible)
  {
    line_strip.color.r = 1.0;
    line_strip.color.g = 1.0;
  }
  else
    line_strip.color.g = 0.9 * (1.0 - std::min(1.0f, cost));
}

template<typename GoalType, typename StateType>
  void CircularTrajectorySetVisualRepresentation<GoalType, StateType>::pushTrajectory(
      const std::vector<rtcus_nav_msgs::DynamicState2D>& trajectory, bool isAdmisible, float cost)
  {
    visualization_msgs::Marker line_strip;
    getDefaultTrajectoryMarker(line_strip);
    line_strip.header.frame_id = this->local_frame_;
    line_strip.ns = "integrated_trajectories";

    for (unsigned int i = 0; i < trajectory.size(); i++)
    {
      geometry_msgs::Point p;
      rtcus_conversions::Conversions::convert(trajectory[i], p);
      line_strip.points.push_back(p);
    }
    decorateTrajectory(line_strip, isAdmisible, cost);
    line_strip.id = trajectory_markers_b.size();
    trajectory_markers_b.push_back(line_strip);
  }

template<typename GoalType, typename StateType>
  void CircularTrajectorySetVisualRepresentation<GoalType, StateType>::getDefaultTrajectoryMarker(
      visualization_msgs::Marker& line_strip)
  {
    line_strip.header.frame_id = this->representation_frame_name_;
    line_strip.header.stamp = ros::Time();
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.ns = "dwa_trajectories";
    line_strip.lifetime = ros::Duration(10.0);

    line_strip.scale.x = 0.005;
    line_strip.scale.y = 0.005;
    line_strip.scale.z = 0.005;
    line_strip.pose.orientation.w = 1.0;
  }

template<typename GoalType, typename StateType>
  void CircularTrajectorySetVisualRepresentation<GoalType, StateType>::represent(
      const rtcus_dwa::DwaLocalPlannerBase<GoalType> & target_object_)

  {
    ROS_DEBUG(" Publishing Trajectories ...");
    const rtcus_dwa::DwaConfig& current_config = target_object_.getConfig();
    t_float v_bottom = current_config.get_v_botom();
    t_float omega_left = current_config.get_omega_left();
    int resolution_width = current_config.get_resolution_width();
    int resolution_height = current_config.get_resolution_height();

    DwaActionSpace& action_space = dynamic_cast<DwaActionSpace&>(*target_object_.action_space_);
    t_float max_angular_velocity = std::max(std::fabs(action_space.front().getAction().angular),
                                            std::fabs(action_space.back().getAction().angular));
    double duration = M_PI / max_angular_velocity;

    visualization_msgs::Marker line_strip;
    getDefaultTrajectoryMarker(line_strip);
    {
      int count = 0;
      t_float v = v_bottom;
      for (int i = 0; i < resolution_height; v += current_config.v_step, i++)
      {
        t_float omega = omega_left;
        for (int j = 0; j < resolution_width; omega += current_config.omega_step, j++, count++)
        {
          const CommandCost<Twist2D> &command = action_space.getAction((i * resolution_width + j));
          if (command.trajectory_ == NULL)
          {
            line_strip.id = trajectory_markers_.size();
            Twist2D action;
            action.linear = v;
            action.angular = omega;

            decorateTrajectory(line_strip, command.isAdmisibleCommand(), (float)command.getTotalCost());
            paint_motion_model_trajectory<t_float, StateType, Twist2D>(line_strip, *this->trajectory_,
                                                                       target_object_.getMotionModel(),
                                                                       this->local_state_, action, duration, 0.0);
            trajectory_markers_.push_back(line_strip);
            line_strip.points.clear();
          }
          else
            this->pushTrajectory(*command.trajectory_, command.isAdmisibleCommand(), command.getTotalCost());
        }
      }
    }
    //------- PAINT BEST TRAJECTORY ----------------
    Marker best_line_strip = line_strip;
    {
      best_line_strip.id = trajectory_markers_.size();
      best_line_strip.color.b = 1.0;
      best_line_strip.color.r = best_line_strip.color.g = 0.5;
      best_line_strip.scale.x *= 2
          * max(5.0,
                10.0 * this->best_command_.linear / current_config.getKinodynamicConfig().linear_forward_speed_limit);

      paint_motion_model_trajectory<t_float, StateType, Twist2D>(best_line_strip, *this->trajectory_,
                                                                 target_object_.getMotionModel(), this->local_state_,
                                                                 this->best_command_, duration, 0.5);
      trajectory_markers_.push_back(best_line_strip);
    }

    VisualRepresetationMarkersBase::publisn_and_clean_collision_marker(this->trajectory_markers_, "dwa_trajectories",
                                                                       this->representation_publisher_,
                                                                       this->last_trajectory_markers_count_,
                                                                       this->representation_frame_name_,
                                                                       visualization_msgs::Marker::LINE_STRIP);

    VisualRepresetationMarkersBase::publisn_and_clean_collision_marker(this->trajectory_markers_b,
                                                                       "integrated_trajectories",
                                                                       this->representation_publisher_,
                                                                       this->last_trajectory_markers_countb_,
                                                                       this->local_frame_,
                                                                       visualization_msgs::Marker::LINE_STRIP);
  }

template class CircularTrajectorySetVisualRepresentation<rtcus_dwa::PointXY, DynamicState2D> ;
template class CircularTrajectorySetVisualRepresentation<rtcus_nav_msgs::Twist2D, DynamicState2D> ;

}
}

