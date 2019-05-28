/*
 * ArbitraryTrajectoryClearanceCircularRobot.h
 *
 *  Created on: Feb 19, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_navigation/trajectory_clearance/workspace_trajectory_2d_circular_robot_clearance.h>
#include <rtcus_conversions/conversions.h>
#include <rtcus_assert/rtcus_assert.h>

namespace rtcus_navigation
{
namespace trajectory_clearance
{
using namespace rtcus_robot_shapes::interfaces;
using namespace std;
using namespace pcl;

WorkspaceTrajectory2DCircularRobotClearance::WorkspaceTrajectory2DCircularRobotClearance() :
    max_distance_(numeric_limits<double>::max())
{

}

WorkspaceTrajectory2DCircularRobotClearance::~WorkspaceTrajectory2DCircularRobotClearance()
{

}

void WorkspaceTrajectory2DCircularRobotClearance::setMaxDistance(double distance)
{
  this->max_distance_ = distance;
}
double WorkspaceTrajectory2DCircularRobotClearance::getMaxDistance() const
{
  return max_distance_;
}

//memory alignment is not required for 2D points
bool WorkspaceTrajectory2DCircularRobotClearance::computeClearance(const vector<pcl::PointXY>& trajectory,
                                                                   const pcl::PointXY& obstacle,
                                                                   const ICircularRobotShape& shape,
                                                                   DefaultClearanceCost& clearance) const
{
  vector<pcl::PointXY> obstacles(1);
  obstacles[0] = obstacle;
  return this->computeClearance(trajectory, obstacles, shape, clearance);
}

bool WorkspaceTrajectory2DCircularRobotClearance::computeClearance(
    const std::vector<rtcus_nav_msgs::Pose2D>& trajectory, const vector<pcl::PointXY>& obstacles,
    const ICircularRobotShape& shape, DefaultClearanceCost& clearance) const
{
  return this->computeClearance_aux(trajectory, obstacles, shape, clearance);
}

bool WorkspaceTrajectory2DCircularRobotClearance::computeClearance(const vector<pcl::PointXY>& trajectory,
                                                                   const vector<pcl::PointXY>& obstacles,
                                                                   const ICircularRobotShape& shape,
                                                                   DefaultClearanceCost& clearance) const
{
  return this->computeClearance_aux(trajectory, obstacles, shape, clearance);
}

bool WorkspaceTrajectory2DCircularRobotClearance::computeClearance(const vector<rtcus_nav_msgs::Pose2D>& trajectory,
                                                                   const pcl::PointCloud<pcl::PointXY>& obstacles,
                                                                   const ICircularRobotShape& shape,
                                                                   DefaultClearanceCost& clearance) const
{
  return this->computeClearance_aux(trajectory, obstacles, shape, clearance);
}

template<typename StateType, typename ObstacleVector>
  bool WorkspaceTrajectory2DCircularRobotClearance::computeClearance_aux(const vector<StateType>& trajectory,
                                                                         const ObstacleVector& obstacles,
                                                                         const ICircularRobotShape& shape,
                                                                         DefaultClearanceCost& clearance) const
  {
    RTCUS_ASSERT(trajectory.size() > 1);
    float total_distance = 0;
    DefaultClearanceCost aux_clearance;
    clearance = DefaultClearanceCost();

    for (unsigned int i = 1; i < trajectory.size(); i++)
    {
      PointXY tmp;
      rtcus_conversions::Conversions::convert(trajectory[i - 1], tmp);
      TwoPointSegment segment_trajectory;
      segment_trajectory.x0 = tmp.x;
      segment_trajectory.y0 = tmp.y;
      rtcus_conversions::Conversions::convert(trajectory[i], tmp);
      segment_trajectory.x1 = tmp.x;
      segment_trajectory.y1 = tmp.y;

      bool collision = this->segment_clearance_.computeClearance(segment_trajectory, obstacles, shape, aux_clearance);
      if (collision)
      {
        clearance.linear_collision_distance_ = total_distance + aux_clearance.linear_collision_distance_;
        clearance.collision_point_ = aux_clearance.collision_point_;
        clearance.collision = true;
        return true;
      }
      else if (total_distance > this->getMaxDistance())
      {
        break;
      }
      else
      {
        total_distance += sqrt(
            pow(segment_trajectory.x1 - segment_trajectory.x0, 2)
                + pow(segment_trajectory.y1 - segment_trajectory.y0, 2));
      }
    }
    clearance.linear_collision_distance_ = this->getMaxDistance();
    return false;
  }
}
}

