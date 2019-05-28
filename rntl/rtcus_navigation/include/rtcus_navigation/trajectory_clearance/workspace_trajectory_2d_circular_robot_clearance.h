/*
 * ArbitraryTrajectoryClearanceCircularRobot.h
 *
 *  Created on: Feb 19, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef ARBITRARYTRAJECTORYCLEARANCECIRCULARROBOT_H_
#define ARBITRARYTRAJECTORYCLEARANCECIRCULARROBOT_H_

#include <rtcus_navigation/trajectory_clearance/trajectory_clearance.h>
#include <rtcus_robot_shapes/circular_robot.h>
#include <rtcus_navigation/trajectory_clearance/linear_trajectory_clearance_circular_shape.h>
#include <rtcus_nav_msgs/Pose2D.h>
#include <rtcus_navigation/trajectory_clearance/default_clearance_cost.h>
#include <pcl/point_types.h>
#include <vector>

namespace rtcus_navigation
{
namespace trajectory_clearance
{
using namespace rtcus_robot_shapes::interfaces;
using namespace std;
using namespace rtcus_nav_msgs;

//TODO: c++ Concepts would be great here. But Interface programming its also ok
class WorkspaceTrajectory2DCircularRobotClearance :
    public TrajectoryClearance<vector<pcl::PointXY>, pcl::PointXY, ICircularRobotShape, DefaultClearanceCost>,
    public TrajectoryClearance<vector<rtcus_nav_msgs::Pose2D>, vector<pcl::PointXY>, ICircularRobotShape,
        DefaultClearanceCost>,
    public TrajectoryClearance<vector<rtcus_nav_msgs::Pose2D>, pcl::PointCloud<pcl::PointXY>, ICircularRobotShape,
        DefaultClearanceCost>,
    public TrajectoryClearance<vector<pcl::PointXY>, vector<pcl::PointXY>, ICircularRobotShape, DefaultClearanceCost>
{
protected:
  double max_distance_;
  pcl::PointXY collision_point_;
  LinearTrajectoryClearance segment_clearance_;

  template<typename StateType, typename ObstacleVector>
    bool computeClearance_aux(const vector<StateType>& trajectory, const ObstacleVector& obstacles,
                              const ICircularRobotShape& shape, DefaultClearanceCost& clearance) const;
public:
  WorkspaceTrajectory2DCircularRobotClearance();
  const PointXY& getCollisionPoint() const;
  virtual ~WorkspaceTrajectory2DCircularRobotClearance();

  //memory alignment is not required for 2D points
  virtual bool computeClearance(const vector<pcl::PointXY>& trajectory, const pcl::PointXY& obstacle,
                                const ICircularRobotShape& shape, DefaultClearanceCost& clearance) const;

  virtual bool computeClearance(const vector<rtcus_nav_msgs::Pose2D>& trajectory, const vector<pcl::PointXY>& obstacles,
                                const ICircularRobotShape& shape, DefaultClearanceCost& clearance) const;

  virtual bool computeClearance(const vector<rtcus_nav_msgs::Pose2D>& trajectory,
                                const pcl::PointCloud<pcl::PointXY>& obstacles, const ICircularRobotShape& shape,
                                DefaultClearanceCost& clearance) const;

  virtual bool computeClearance(const vector<pcl::PointXY>& trajectory, const vector<pcl::PointXY>& obstacles,
                                const ICircularRobotShape& shape, DefaultClearanceCost& clearance) const;

  virtual void setMaxDistance(double distance);
  virtual double getMaxDistance() const;
}
;
}
}

#endif /* ARBITRARYTRAJECTORYCLEARANCECIRCULARROBOT_H_ */
