/*
 * trajectory_distance_heading_cost_strategy.cpp
 *
 *  Created on: Nov 25, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_dwa/heading_strategies/trajectory_goal_intersection_heading_strategy.h>
#include <rtcus_dwa/common.h>

namespace rtcus_dwa
{
namespace heading_strategies
{

double goal_to_trajectory_distance(const pcl::PointXY& goal, const Twist2D& action)
{
  //circular trajectory
  if (fabs(action.angular) > DWA_MAXIMUM_RADIOUS_TO_AVOID_NUMERICAL_INESTABILITIES)
  {
    double r = action.linear / action.angular;
    double icr_y = r;
    double diff_y = (goal.y - icr_y);

    //UNDERSTAND THIS: this is the distance from the icr to the goal
    double goal_to_icr_dist = sqrt(goal.x * goal.x + diff_y * diff_y);
    return fabs((goal_to_icr_dist - fabs(r)));
  }
  //straight trajectory
  else
  {
    return fabs(goal.y);
  }
}

t_float TragectoryGoalIntersectionHeadingCostStrategy::computeCost(const CommandCost<Twist2D>& action)
{
  t_float cost = goal_to_trajectory_distance(*(this->goal_), action.getAction());
  this->normalize(cost);
  return cost;

  //-------------- TAKE A SUITABLE GOAL ATTRACTION POINT ----------------------
  /*double goal_distance = sqrt(goal.x * goal.x + goal.y * goal.y);
   double goal_x, goal_y;
   if (goal_distance > config.max_collision_distance)
   {
   goal_x = config.max_collision_distance * goal.x / goal_distance;
   goal_y = config.max_collision_distance * goal.y / goal_distance;
   }
   else
   {
   goal_x = goal.x;
   goal_y = goal.y;
   }*/

  //------------- COMPUTE THE LIMIT DISTANCES FOR NORMALIZATION ------------------------
  /*double max_goal_distance, r_goal_distance, l_goal_distance;
   r_goal_distance = goal_to_trajectory_distance(action, goal);

   Twist2D limit_action;
   limit_action.angular = config.get_omega_left();

   //to make the normalization get the normal distances of the most extreme trajectories and take the maxium
   if (fabs(config.get_omega_left()) > DWA_MAXIMUM_RADIOUS_TO_AVOID_NUMERICAL_INESTABILITIES)
   {
   r_left = config.get_v_botom() / config.get_omega_left();
   double icr_dist = sqrt(pow(goal_y - r_left, 2) + goal_x * goal_x);
   l_goal_distance = icr_dist - config.obstacle_inflation;
   }
   else
   {
   r_left = numeric_limits<double>::max();
   l_goal_distance = sqrt(goal_y * goal_y);
   }

   if (fabs(config.get_omega_right()) > DWA_MAXIMUM_RADIOUS_TO_AVOID_NUMERICAL_INESTABILITIES)
   {
   r_right = config.get_v_botom() / config.get_omega_right();
   double icr_dist = sqrt(pow(goal_y - r_right, 2) + goal_x * goal_x);
   r_goal_distance = icr_dist - config.obstacle_inflation;
   }
   else
   {
   r_right = numeric_limits<double>::max();
   r_goal_distance = sqrt(goal_y * goal_y);
   }
   max_goal_distance = max(r_goal_distance, l_goal_distance);
   */
  // --------------- COMPUTE THE HEADING COST ---------------------------------------------
  /*
   double cost;
   if (fabs(action.angular) > DWA_MAXIMUM_RADIOUS_TO_AVOID_NUMERICAL_INESTABILITIES)
   {
   double r = action.linear / action.angular;
   double icr_y = r;
   double diff_y = (goal_y - icr_y);

   //UNDERSTAND THIS: this is the distance from the icr to the goal
   double goal_to_icr_dist = sqrt(goal_x * goal_x + diff_y * diff_y);

   //ROS_INFO(
   //   "v %lf omega %lf -> dist (%lf) ---  max goal distance (normalization_factor): %lf", v, omega, dist, max_goal_distance);
   if (max_goal_distance == 0 && goal_to_icr_dist == 0)
   return 1.0;

   else if (max_goal_distance == 0 && goal_to_icr_dist != 0)
   return 0.0;
   else
   {
   //distance of the trajectory to the goal
   cost = fabs((goal_to_icr_dist - fabs(r))) / max_goal_distance;

   //ROS_INFO(
   //    " IRY %lf goalx %lf goal y %lf -> v %lf omega %lf arc distance %lf -> cost %lf", icr_y, goal_x, goal_y, v, omega, dist, cost);
   //distance of the trajectory to the goal

   }
   }
   else
   {
   cost = fabs(goal_y) / max_goal_distance;
   }

   cost = max(min(1.0, cost), 0.0);
   */

}

}
}
