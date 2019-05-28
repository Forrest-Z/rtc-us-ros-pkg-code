/*
 * geometry_trajectory_collision_visual_representation.h
 *
 *  Created on: Oct 25, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef GEOMETRY_TRAJECTORY_COLLISION_VISUAL_REPRESENTATION_H_
#define GEOMETRY_TRAJECTORY_COLLISION_VISUAL_REPRESENTATION_H_

#include <rtcus_navigation_tools/visual_representations/visual_representation_markers_set_base.h>
#include <rtcus_dwa/common.h>

namespace rtcus_dwa
{
namespace visual_representation
{
using namespace rtcus_navigation_tools;
using namespace visualization_msgs;

template<typename GoalType>
  class ObstacleInfoVisualizationRepresentation : public VisualRepresetationMarkersBase
  {
  protected:
    std::list<Marker> collision_representation_markers_;
  public:
    virtual ~ObstacleInfoVisualizationRepresentation()
    {

    }
    ObstacleInfoVisualizationRepresentation(const std::string& representation_frame, ros::NodeHandle& n) :
        VisualRepresetationMarkersBase::VisualRepresetationMarkersBase(representation_frame, "collision_points", n)
    {

    }

    virtual void push_marker(const Marker& marker)
    {
      collision_representation_markers_.push_back(marker);
    }

    void paintCollisions(const rtcus_dwa::DwaLocalPlannerBase<GoalType> & target_object_)
    {

      for (unsigned int i = 0; i < target_object_.action_space_->size(); i++)
      {
        const CommandCost<Twist2D> &command = target_object_.action_space_->getAction(i);

        if (command.collision)
        {
          visualization_msgs::Marker m;
          const PointXY& p = command.collision_point_;

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
          collision_representation_markers_.push_back(m);
        }
      }
    }

    void paintArea(const rtcus_dwa::DwaLocalPlannerBase<GoalType> & target_object)
    {
      visualization_msgs::Marker m;
      m.type = visualization_msgs::Marker::CYLINDER;
      m.pose.position.x = 0.0;
      m.pose.position.y = 0.0;
      m.pose.position.z = 0.00;
      m.pose.orientation.w = 1.0;

      m.scale.x = target_object.command_clearance_->get_worse_braking_distance() / 2.0;
      m.scale.y = target_object.command_clearance_->get_worse_braking_distance() / 2.0;
      m.scale.z = 0.03;

      m.color.b = 0.1;
      m.color.g = 0.1;
      m.color.r = 1.0;
      m.color.a = 0.1;
      m.ns = "max_collision_area";
      collision_representation_markers_.push_back(m);
    }
    void represent(const rtcus_dwa::DwaLocalPlannerBase<GoalType> & target_object_)
    {
      this->paintArea(target_object_);
      this->paintCollisions(target_object_);
      VisualRepresetationMarkersBase::push_markers(collision_representation_markers_);
      VisualRepresetationMarkersBase::publish_all_markers();
      collision_representation_markers_.clear();
    }
  };
}
}

#endif /* GEOMETRY_TRAJECTORY_COLLISION_VISUAL_REPRESENTATION_H_ */
