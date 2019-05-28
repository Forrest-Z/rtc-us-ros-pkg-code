/*
 * circular_trajectory_set_visual_representation.h
 *
 *  Created on: Oct 25, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef CIRCULAR_TRAJECTORY_SET_VISUAL_REPRESENTATION_H_
#define CIRCULAR_TRAJECTORY_SET_VISUAL_REPRESENTATION_H_

#include <rtcus_navigation_tools/visual_representations/visual_representation_markers_set_base.h>
#include <rtcus_nav_msgs/Twist2D.h>
#include <rtcus_nav_msgs/DynamicState2D.h>
#include <rtcus_dwa/common.h>

namespace rtcus_dwa
{
namespace visual_representation
{
using namespace rtcus_navigation_tools;

template<typename GoalType, typename StateType>
  class CircularTrajectorySetVisualRepresentation : public VisualRepresetationMarkersBase
  {
  private:
    std::string local_frame_;
    std::list<visualization_msgs::Marker> trajectory_markers_;
    unsigned long last_trajectory_markers_count_;

    std::list<visualization_msgs::Marker> trajectory_markers_b;
    unsigned long last_trajectory_markers_countb_;

    rtcus_nav_msgs::Twist2D best_command_;
    StateType local_state_;
    GoalType local_goal_;
    boost::shared_ptr<std::vector<StateType> > trajectory_;
    void getDefaultTrajectoryMarker(visualization_msgs::Marker& line_strip);

  public:
    virtual ~CircularTrajectorySetVisualRepresentation();
    CircularTrajectorySetVisualRepresentation(const std::string& representation_frame, const std::string& local_frame,
                                              ros::NodeHandle& n);
    void updateCommand(const rtcus_nav_msgs::Twist2D& best_command, const StateType& local_state,
                       const GoalType& local_goal);

    void pushTrajectory(const std::vector<rtcus_nav_msgs::DynamicState2D>& trajectory, bool isAdmisible, float cost);
    virtual void represent(const rtcus_dwa::DwaLocalPlannerBase<GoalType> & target_object_);
  };
}
}

#endif /* CIRCULAR_TRAJECTORY_SET_VISUAL_REPRESENTATION_H_ */
