/*
 * paint_trajectory.h
 *
 *  Created on: Nov 4, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef PAINT_TRAJECTORY_H_
#define PAINT_TRAJECTORY_H_

#include <rtcus_motion_models/motion_models/non_holonomic_2d.h>
#include <rtcus_motion_models/motion_models/motion_models_impl.h>
#include <rtcus_conversions/conversions.h>
#include <visualization_msgs/Marker.h>
#include <vector>

namespace rtcus_navigation_tools
{
template<typename t_float, typename StateType, typename ActionType>
  void paint_motion_model_trajectory(visualization_msgs::Marker& line_strip, std::vector<StateType>& trajectory,
                                     const rtcus_motion_models::MotionModel<StateType, ActionType>& motion_model,
                                     const StateType& state, const ActionType& action, double duration, double zorder =
                                         0.0)
  {
    //PAINT TRAJECTORY LIMITS
    motion_model.sampleStates(state, action, ros::Duration(duration), trajectory);

    //ITERATE POINTS
    for (unsigned int i = 0; i < trajectory.size(); i++)
    {
      geometry_msgs::Point p;
      rtcus_conversions::Conversions::convert(trajectory[i], p);
      line_strip.points.push_back(p);
    }
  }
}

#endif /* PAINT_TRAJECTORY_H_ */
