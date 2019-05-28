/*
 * dwa_algorithm_representation.h
 *
 *  Created on: Oct 24, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef DWA_ALGORITHM_REPRESENTATION_H_
#define DWA_ALGORITHM_REPRESENTATION_H_

#include <rtcus_dwa/common.h>
#include <rtcus_dwa/visual_representation/geometry_trajectory_collision_visual_representation.h>
#include <rtcus_dwa/visual_representation/dynamic_window_action_space_visual_representation.h>
#include <rtcus_dwa/visual_representation/circular_trajectory_set_visual_representation.h>
#include <rtcus_navigation_tools/visual_representations/visual_representation.h>

namespace rtcus_dwa
{

namespace visual_representation
{
using namespace rtcus_navigation_tools;

template<typename GoalType>
  class DwaVisualRepresentation : public rtcus_navigation_tools::VisualRepresetationBase
  {

    ObstacleInfoVisualizationRepresentation<GoalType> obstacle_info_representation;
    DynamicWindowActionSpaceRepresentation<GoalType> action_space_image_representation;
    CircularTrajectorySetVisualRepresentation<GoalType, rtcus_nav_msgs::DynamicState2D> trajectory_action_space_representation;
  public:
    virtual ~DwaVisualRepresentation()
    {
    }

    DwaVisualRepresentation(DwaLocalPlannerBase<GoalType>& target, const std::string& state_prediction_frame,
                            const std::string& global_frame, ros::NodeHandle n) :
        rtcus_navigation_tools::VisualRepresetationBase(state_prediction_frame, n), obstacle_info_representation(
            state_prediction_frame, n), action_space_image_representation(state_prediction_frame, n), trajectory_action_space_representation(
            global_frame, state_prediction_frame, n)
    {
      ROS_DEBUG(" DWA REPRESENTATION: Building");

      //FOR EACH COMMAND CALL THIS METHOD
      target.command_clearance_->onVisualRepresentation.connect(
          boost::bind(&ObstacleInfoVisualizationRepresentation<GoalType>::push_marker, &obstacle_info_representation,
                      _1));

      //FOREACH PLANNING ITERATION CALL THIS METHOD
      target.onPostComputeCommand.connect(
          boost::bind(&DwaVisualRepresentation<GoalType>::onRepresent, this, _1, _2, _3, _4));

      target.onActionEvaluated.connect(
          boost::bind(&DwaVisualRepresentation<GoalType>::onTrajectoryEvaluated, this, _1));

      ROS_DEBUG(" DWA REPRESENTATION: Done");
    }

  protected:
    typedef typename rtcus_dwa::DwaLocalPlannerBase<GoalType>::TNavigationPlanner TNavigationPlanner;

    void onTrajectoryEvaluated(const CommandCost<Twist2D>& command_cost)
    {
      if (command_cost.trajectory_ != NULL)
        this->trajectory_action_space_representation.pushTrajectory(*(command_cost.trajectory_),
                                                                    command_cost.isAdmisibleCommand(),
                                                                    command_cost.getTotalCost());
    }

    void onRepresent(TNavigationPlanner& sender, const rtcus_nav_msgs::Twist2D& best_cmd,
                     const DynamicState2D& local_state, const GoalType& local_goal)
    {
      this->action_space_image_representation.update(best_cmd);
      this->trajectory_action_space_representation.updateCommand(best_cmd, local_state, local_goal);
      rtcus_dwa::DwaLocalPlannerBase<GoalType>& target_object =
          dynamic_cast<rtcus_dwa::DwaLocalPlannerBase<GoalType>&>(sender);
      this->represent(target_object);
    }

    virtual void represent(const rtcus_dwa::DwaLocalPlannerBase<GoalType> & target_object)
    {
      this->obstacle_info_representation.represent(target_object);
      this->action_space_image_representation.represent(target_object);
      this->trajectory_action_space_representation.represent(target_object);
    }


  };
}

}
#endif /* DWA_ALGORITHM_REPRESENTATION_H_ */
