/*
 * shared_control_goal_port.cpp
 *
 *  Created on: Jan 21, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_navigation/goal_ports/waypoints_goal_port.h>
#include <boost/smart_ptr.hpp>

namespace rtcus_navigation
{
namespace goal_ports
{

using namespace geometry_msgs;
WayPointsGoalPort::~WayPointsGoalPort()
{
}

void WayPointsGoalPort::init(const std::string& reference_frame, const std::string& robot_base_frame)
{
  DefaultMsgsGoalPortBase<pcl::PointXY>::init(reference_frame, robot_base_frame);

}

void WayPointsGoalPort::topic_subscribe()
{
  PoseStamped initial_goal;
  current_goal_index_ = -1;
  initial_goal.header.frame_id = this->getReferenceFrame();
  initial_goal.header.stamp = ros::Time::now();
  this->process_simple_goal(initial_goal);
  ros::NodeHandle& nh = this->node_;
  this->goal_base_sub_ = nh.subscribe<PoseStamped>("goal", 1,
                                                   boost::bind(&WayPointsGoalPort::goalMsgCallback2, this, _1));

  this->goal_sub_ = nh.subscribe<nav_msgs::Path>("waypoints_goal", 1,
                                                 boost::bind(&WayPointsGoalPort::goalMsgCallback, this, _1));

}

void WayPointsGoalPort::process_simple_goal(const PoseStamped& goal)
{
  nav_msgs::Path p;
  p.poses.push_back(goal);
  p.header = goal.header;
  this->setWayPoints(p);
  this->moveNextWayPoint();
}
void WayPointsGoalPort::goalMsgCallback2(const boost::shared_ptr<const PoseStamped>& goal)
{
  this->process_simple_goal(*goal);
}

void WayPointsGoalPort::goalMsgCallback(const boost::shared_ptr<const nav_msgs::Path>& path_goal)
{
  this->setWayPoints(*path_goal);
  this->moveNextWayPoint();
}

void WayPointsGoalPort::setWayPoints(const nav_msgs::Path& path)
{
  this->mission_path_ = path;
}

void WayPointsGoalPort::moveNextWayPoint()
{
  std::string goal_frame;
  ros::Time goal_time;
  if (this->mission_path_.poses.size() > 0)
  {
    current_goal_index_ = (current_goal_index_ + 1) % this->mission_path_.poses.size();
    if (current_goal_index_ >= 0 && current_goal_index_ < (int)this->mission_path_.poses.size())
    {
      PointXY subgoal;
      rtcus_conversions::Conversions::convert(this->mission_path_.poses[this->current_goal_index_], subgoal);
      StampedData<pcl::PointXY> goal(subgoal, this->mission_path_.header.stamp, this->mission_path_.header.frame_id);
      ROS_INFO_STREAM(
          "GOAL RECEIVED << "<< *goal<<" "<<this->mission_path_.header.stamp<<" "<<this->mission_path_.header.frame_id);
      this->pushGoal(goal);
    }
  }
}
}
}
