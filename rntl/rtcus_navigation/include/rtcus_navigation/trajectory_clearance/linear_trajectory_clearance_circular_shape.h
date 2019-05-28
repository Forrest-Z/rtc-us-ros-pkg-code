/*
 * LinearTrajectoryClearanceCircularShape.h
 *
 *  Created on: Apr 8, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef LINEARTRAJECTORYCLEARANCECIRCULARSHAPE_H_
#define LINEARTRAJECTORYCLEARANCECIRCULARSHAPE_H_
#include <math.h>
#include <rtcus_navigation/trajectory_clearance/trajectory_clearance.h>
#include <rtcus_robot_shapes/interfaces/circular.h>
#include <rtcus_navigation/trajectory_clearance/default_clearance_cost.h>
#include <pcl/point_types.h>

namespace rtcus_navigation
{
namespace trajectory_clearance
{
using namespace rtcus_robot_shapes::interfaces;
using namespace std;
using namespace pcl;

class TwoPointSegment
{
public:
  double x0;
  double y0;
  double x1;
  double y1;
};

/*\brief This class implements the intersection between a line and a circle*/
class LinearTrajectoryClearance : public TrajectoryClearance<TwoPointSegment, pcl::PointXY, ICircularRobotShape,
                                      DefaultClearanceCost>,
                                  public TrajectoryClearance<TwoPointSegment, pcl::PointCloud<pcl::PointXY>,
                                      ICircularRobotShape, DefaultClearanceCost>,
                                  public TrajectoryClearance<TwoPointSegment, std::vector<pcl::PointXY>,
                                      ICircularRobotShape, DefaultClearanceCost>
{
public:
  LinearTrajectoryClearance();
  virtual ~LinearTrajectoryClearance();
  virtual bool computeClearance(const TwoPointSegment& action, const vector<pcl::PointXY>& obstacles,
                                const ICircularRobotShape& shape, DefaultClearanceCost& clearance) const;

  virtual bool computeClearance(const TwoPointSegment& action, const pcl::PointCloud<pcl::PointXY>& obstacles,
                                const ICircularRobotShape& shape, DefaultClearanceCost& clearance) const;

  virtual bool computeClearance(const TwoPointSegment& action, const pcl::PointXY& obstacle,
                                const ICircularRobotShape& shape, DefaultClearanceCost& clearance) const;

  virtual void setMaxDistance(double distance);
  virtual double getMaxDistance() const;

protected:
  double max_distance_;
  inline void getfixedVariables(const TwoPointSegment& action, const ICircularRobotShape& shape, double& r,
                                double& r_sq, double& vx, double& vy, double& bounding_xmin, double& bounding_xmax,
                                double& bounding_ymin, double& bounding_ymax, double& xa, double& ya, double& xb,
                                double& yb, double& D, double& D_sq, double& dr, double& dr_sq) const;

  inline double computeDistance(const TwoPointSegment& action, const ICircularRobotShape& shape, const PointXY& o,
                                double& r, double& r_sq, double& vx, double& vy, double& bounding_xmin,
                                double& bounding_xmax, double& bounding_ymin, double& bounding_ymax, double& xa,
                                double& ya, double& xb, double& yb, double& D, double& D_sq, double& dr, double& dr_sq,
                                pcl::PointXY& clearance) const;

  template<typename ObstacleVectorLike>
    inline bool computeClerance_aux(const TwoPointSegment& action, const ObstacleVectorLike& obstacles,
                                    const ICircularRobotShape& shape, DefaultClearanceCost& clearance) const;

}
;
}
}

#endif /* LINEARTRAJECTORYCLEARANCECIRCULARSHAPE_H_ */
