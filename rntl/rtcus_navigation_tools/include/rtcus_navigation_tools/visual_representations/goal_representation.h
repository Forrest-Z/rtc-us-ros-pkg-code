/*
 * goal_representation.h
 *
 *  Created on: Oct 25, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef GOAL_REPRESENTATION_H_
#define GOAL_REPRESENTATION_H_

#include <rtcus_navigation_tools/visual_representations/visual_representation.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <ros/ros.h>
#include <rtcus_navigation_tools/visual_representations/goal_representation.h>
#include <rtcus_navigation_tools/visual_representations/visual_representation_markers_set_base.h>
#include <rtcus_nav_msgs/Twist2D.h>
#include <rtcus_nav_msgs/Pose2D.h>
#include <rtcus_conversions/conversions.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <rtcus_motion_models/motion_models/non_holonomic_2d.h>
#include <rtcus_navigation_tools/visual_representations/paint_trajectory.h>

namespace rtcus_navigation_tools
{
namespace visual_representations
{
using namespace std;
/**
 *  \brief Visual represention for a type object as a goal
 *  */
template<typename GoalType>
  class GoalVisualRepresentation
  {
  public:
    virtual ~GoalVisualRepresentation();

    GoalVisualRepresentation(std::string representation_frame_name, ros::NodeHandle& n);
    /**
     * \brief This is the template method that has to be overriden by template specialization
     * */
    void represent(const GoalType& target_object);
  };

template<>
  class GoalVisualRepresentation<pcl::PointXY> : public VisualRepresetationBase
  {
    tf::TransformBroadcaster broadcaster_;
  public:
    virtual ~GoalVisualRepresentation()
    {

    }

    GoalVisualRepresentation(std::string representation_frame_name, ros::NodeHandle& n) :
        VisualRepresetationBase(representation_frame_name, n)
    {

    }

    virtual void represent(const pcl::PointXY& goal)
    {
      tf::StampedTransform transf;
      rtcus_conversions::Conversions::convert(goal, (tf::Transform&)transf);
      transf.stamp_ = ros::Time::now();
      transf.frame_id_ = this->representation_frame_name_;
      transf.child_frame_id_ = "estimated_goal";
      broadcaster_.sendTransform(transf);
    }
  };

template<>
  class GoalVisualRepresentation<rtcus_nav_msgs::Twist2D> : public VisualRepresetationMarkersBase
  {
  private:
    rtcus_motion_models::DeterministicNonHolonomic2D<rtcus_nav_msgs::Pose2D, rtcus_nav_msgs::Twist2D> circular_motion_model;
    std::vector<rtcus_nav_msgs::Pose2D> trajectory_;
    std::list<visualization_msgs::Marker> line_strip_;
  public:
    virtual ~GoalVisualRepresentation()
    {

    }

    GoalVisualRepresentation(std::string representation_frame_name, ros::NodeHandle& n) :
        VisualRepresetationMarkersBase(representation_frame_name, "goal_representation", n), trajectory_(40)
    {

      visualization_msgs::Marker line_strip;
      line_strip.header.frame_id = this->representation_frame_name_;
      line_strip.type = visualization_msgs::Marker::LINE_STRIP;
      line_strip.lifetime = ros::Duration(10.0);
      line_strip.ns = "trajectory_goal";

      line_strip.scale.x = 0.02;
      line_strip.scale.y = 0.02;
      line_strip.scale.z = 0.02;
      line_strip.pose.orientation.w = 1.0;
      line_strip.color.b = 1.0;
      line_strip.color.g = 0.4;
      line_strip.color.r = 1.0;
      line_strip.color.a = 1.0;
      line_strip_.push_back(line_strip);

      visualization_msgs::Marker final_sphere = line_strip;
      final_sphere.type = visualization_msgs::Marker::SPHERE;
      line_strip_.push_back(final_sphere);
    }

    void represent(const rtcus_nav_msgs::Twist2D& goal)
    {
      line_strip_.front().header.stamp = ros::Time();
      rtcus_nav_msgs::Pose2D original_local_pose;
      original_local_pose.x = original_local_pose.y = original_local_pose.phi = 0;

      line_strip_.front().points.clear();
      line_strip_.front().scale.x = 0.02 + goal.linear / 500.0;

      double time = 0.2; //seconds
      double omega = fabs(goal.angular);
      if (omega > 0.01)
        time = M_PI / omega;
      else
        time = M_PI / 0.01;

      paint_motion_model_trajectory<double, rtcus_nav_msgs::Pose2D, rtcus_nav_msgs::Twist2D>(this->line_strip_.front(),
                                                                                             trajectory_,
                                                                                             circular_motion_model,
                                                                                             original_local_pose, goal,
                                                                                             time, 0.3);
      line_strip_.back().pose.position = line_strip_.front().points.back();
      line_strip_.back().pose.orientation.w = 1.0;
      line_strip_.back().scale.x = 2.0 * line_strip_.front().scale.x;
      line_strip_.back().scale.y = 2.0 * line_strip_.front().scale.x;
      line_strip_.back().scale.z = 2.0 * line_strip_.front().scale.x;

      this->push_markers(line_strip_);
      this->publish_all_markers();
    }
  };

}

}

#endif /* GOAL_REPRESENTATION_H_ */
