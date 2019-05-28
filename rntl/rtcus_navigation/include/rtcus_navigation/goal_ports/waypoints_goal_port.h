/*
 * goal_port_base.h
 *
 *  Created on: Jun 20, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef WAYPOINTS_GOAL_PORT_H_
#define WAYPOINTS_GOAL_PORT_H_

#include <rtcus_navigation/common.h>
#include <rtcus_navigation/goal_port.h>
#include <rtcus_navigation/goal_ports/goal_port_base.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <rtcus_assert/rtcus_assert.h>
#include <rtcus_navigation/reached_goal_predicate.h>

namespace rtcus_navigation
{
namespace goal_ports
{
class WayPointsGoalPort : public rtcus_navigation::goal_ports::DefaultMsgsGoalPortBase<pcl::PointXY>
{
protected:
  ros::Subscriber goal_sub_;
  ros::Subscriber goal_base_sub_;
  nav_msgs::Path mission_path_;
  int current_goal_index_;
  virtual void topic_subscribe();
  void process_simple_goal(const geometry_msgs::PoseStamped& goal);
  virtual void init(const std::string& reference_frame, const std::string& robot_base_frame);
  void goalMsgCallback2(const boost::shared_ptr<const geometry_msgs::PoseStamped>& goal);
  void goalMsgCallback(const boost::shared_ptr<const nav_msgs::Path>& path_goal);
  void moveNextWayPoint();
  void setWayPoints(const nav_msgs::Path& path);

public:

  virtual ~WayPointsGoalPort();
  template<typename TNavigationNode>
    WayPointsGoalPort(boost::shared_ptr<TNavigationNode>& node) :
        DefaultMsgsGoalPortBase<pcl::PointXY>::DefaultMsgsGoalPortBase()
    {
      node->onGoalReached.connect(boost::bind(&WayPointsGoalPort::moveNextWayPoint, this));
    }
};
}
}

#endif /* GOAL_PORT_BASE_H_ */
