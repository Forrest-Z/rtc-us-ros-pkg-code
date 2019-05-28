/*
 * linear_trajectory_clearance.cpp
 *
 *  Created on: Apr 8, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */
#include <rtcus_navigation/trajectory_clearance/linear_trajectory_clearance_circular_shape.h>
#include <rtcus_assert/rtcus_assert.h>

namespace rtcus_navigation
{
namespace trajectory_clearance
{
using namespace pcl;

LinearTrajectoryClearance::LinearTrajectoryClearance() :
    max_distance_(numeric_limits<double>::max())
{
}
LinearTrajectoryClearance::~LinearTrajectoryClearance()
{
}

void LinearTrajectoryClearance::setMaxDistance(double distance)
{
  this->max_distance_ = distance;
}
double LinearTrajectoryClearance::getMaxDistance() const
{
  return this->max_distance_;
}

/*
 inline void LinearTrajectoryClearance::getfixedVariablesB(const TwoPointSegment& action,
 const ICircularRobotShape& shape, double& r, double& r_sq,
 double& vx, double& vy, double& vxy, double& vx_pw2,
 double& vy_pw2, double& bounding_xmin, double& bounding_xmax,
 double& bounding_ymin, double& bounding_ymax, double& x0,
 double& y0, double& x0_sq, double& x1_sq, double& y0_sq,
 double& y1_sq, double& denominator, double& discriminant_base,
 double& ax, double& ay, double& axy, double& ax2) const
 {
 r = shape.getRadius();
 r_sq = r * r;

 //segment info
 vx = action.x1 - action.x0;
 vy = action.y1 - action.y0;
 vxy = vx * vy;
 vx_pw2 = vx * vx;
 vy_pw2 = vy * vy;

 bounding_xmin = min(action.x0, action.x1) - r;
 bounding_xmax = max(action.x0, action.x1) + r;
 bounding_ymin = min(action.y0, action.y1) - r;
 bounding_ymax = max(action.y0, action.y1) + r;

 x0 = action.x0;
 y0 = action.y0;
 x0_sq = action.x0 * action.x0;
 x1_sq = action.x1 * action.x1;
 y0_sq = action.y0 * action.y0;
 y1_sq = action.y1 * action.y1;
 denominator = (vx_pw2 - vy_pw2);

 //ORIGINAL:
 //double discriminant = r_sq * vx_pw2 - r_sq * vy_pw2 + vx_pw2 * y0_sq - 2 * vx_pw2 * y0 * y_c + vx_pw2 * yc_sq
 //- 2 * vxy * x0 * y0 + 2 * vxy * x0 * y_c + 2 * vxy * x_c * y0 - 2 * vxy * x_c * y_c + vy_pw2 * x0_sq
 //- 2 * vy_pw2 * x0 * x_c + vy_pw2 * xc_sq;
 discriminant_base = r_sq * vx_pw2 - r_sq * vy_pw2 + vx_pw2 * y0_sq - 2 * vxy * x0 * y0;
 ax = 2 * vxy * y0 - 2 * vy_pw2 * x0;
 ay = 2 * vxy * x0 - 2 * vx_pw2 * y0;
 axy = -2 * vxy;
 ax2 = vy_pw2 * x0_sq + vy_pw2;
 }
 */
#define sign(x) (x>=0)?1.0:-1.0

inline void LinearTrajectoryClearance::getfixedVariables(const TwoPointSegment& action,
                                                         const ICircularRobotShape& shape, double& r, double& r_sq,
                                                         double& vx, double& vy, double& bounding_xmin,
                                                         double& bounding_xmax, double& bounding_ymin,
                                                         double& bounding_ymax, double& xa, double& ya, double& xb,
                                                         double& yb, double& D, double& D_sq, double& dr,
                                                         double& dr_sq) const
{
  bounding_xmin = min(action.x0, action.x1) - r;
  bounding_xmax = max(action.x0, action.x1) + r;
  bounding_ymin = min(action.y0, action.y1) - r;
  bounding_ymax = max(action.y0, action.y1) + r;
  vx = action.x1 - action.x0;
  vy = action.y1 - action.y0;

  r = shape.getRadius();
  r_sq = r * r;

}

inline double evaluate_segment(double vx, double vy, double x0, double y0, double t, float&x, float& y)
{
  x = vx * t + x0;
  y = vy * t + y0;

  return sqrt(pow(x - x0, 2) + pow(y - y0, 2));
}

inline double LinearTrajectoryClearance::computeDistance(const TwoPointSegment& action,
                                                         const ICircularRobotShape& shape, const PointXY& o, double& r,
                                                         double& r_sq, double& vx, double& vy, double& bounding_xmin,
                                                         double& bounding_xmax, double& bounding_ymin,
                                                         double& bounding_ymax, double& xa, double& ya, double& xb,
                                                         double& yb, double& D, double& D_sq, double& dr, double& dr_sq,
                                                         pcl::PointXY& collision_point) const
{
  if (action.x0 > bounding_xmax || action.x0 < bounding_xmin || action.y0 > bounding_ymax || action.y0 < bounding_ymin)
  {

    collision_point.x = std::numeric_limits<double>::quiet_NaN();
    collision_point.y = std::numeric_limits<double>::quiet_NaN();
    return this->max_distance_;
  }
  else
  {
    float a = vx * vx + vy * vy;
    float b = 2 * vx * action.x0 + 2 * vy * action.y0 - 2 * vx * o.x - 2 * vy * o.y;
    float c = action.x0 * action.x0 + o.x * o.x - 2 * o.x * action.x0 + action.y0 * action.y0 + o.y * o.y
        - 2 * o.y * action.y0 - r_sq;

    float discriminant = b * b - 4 * a * c;
    if (discriminant < 0)
    {
      collision_point.x = std::numeric_limits<double>::quiet_NaN();
      collision_point.y = std::numeric_limits<double>::quiet_NaN();
      return this->max_distance_;
    }
    else
    {
      float bfactor = -b / (2 * a);
      discriminant = sqrt(discriminant) / (2 * a);
      float t0 = bfactor + discriminant;
      float t1 = bfactor - discriminant;

      bool t0_valid = t0 >= 0 && t0 <= 1;
      bool t1_valid = t1 >= 0 && t1 <= 1;
      if (t0_valid && t1_valid)
      {
        if (t0 <= t1)
        {
          return evaluate_segment(vx, vy, action.x0, action.y0, t0, collision_point.x, collision_point.y);
        }
        else
        {
          return evaluate_segment(vx, vy, action.x0, action.y0, t1, collision_point.x, collision_point.y);
        }
      }
      else if (t0_valid)
      {
        return evaluate_segment(vx, vy, action.x0, action.y0, t0, collision_point.x, collision_point.y);
      }
      else if (t1_valid)
      {
        return evaluate_segment(vx, vy, action.x0, action.y0, t1, collision_point.x, collision_point.y);
      }
      else
      {
        collision_point.x = std::numeric_limits<double>::quiet_NaN();
        collision_point.y = std::numeric_limits<double>::quiet_NaN();

        return this->max_distance_;
      }
    }

  }
}
/*
 inline double LinearTrajectoryClearance::computeDistance(const TwoPointSegment& action,
 const ICircularRobotShape& shape, const PointXY& o, double& r,
 double& r_sq, double& vx, double& vy, double& bounding_xmin,
 double& bounding_xmax, double& bounding_ymin,
 double& bounding_ymax, double& xa, double& ya, double& xb,
 double& yb, double& D, double& D_sq, double& dr,
 double& dr_sq) const
 {
 double x_c = o.x;
 double y_c = o.y;

 //BROAD CHECKING
 if (x_c > bounding_xmax || x_c < bounding_xmin || y_c > bounding_ymax || y_c < bounding_ymin)
 return this->max_distance_;

 //http://mathworld.wolfram.com/Circle-LineIntersection.html
 //double xc_sq = x_c * x_c;
 //double yc_sq = y_c * y_c;

 //segment info
 double lx0 = action.x0 - o.x;
 double ly0 = action.y0 - o.y;

 double lx1 = action.x1 - o.x;
 double ly1 = action.y1 - o.y;

 vx = lx1 - lx0;
 vy = ly1 - ly0;
 dr_sq = vx * vx + vy * vy;
 dr = sqrt(dr_sq);

 D = lx0 * ly1 - lx1 * ly0;
 D_sq = D * D;
 xa = D * vy / dr_sq;
 xb = sign(vy) * vx / dr_sq;
 ya = (-D * vx) / dr_sq;
 yb = fabs(vy) / dr_sq;

 double discriminant = r_sq * dr_sq - D_sq;
 if (discriminant < 0) //NO COLLISION
 {
 return this->max_distance_;
 }
 else
 {
 double discriminant_sqrt = sqrt(discriminant);

 double x0 = xa + xb * discriminant_sqrt;
 double x1 = xa - xb * discriminant_sqrt;

 double y0 = ya + yb * discriminant_sqrt;
 double y1 = ya - yb * discriminant_sqrt;

 double t0, t1;

 if (vx != 0)
 {
 t0 = (x0 - lx0) / vx;
 t1 = (x1 - lx0) / vx;
 }
 else if (vy != 0)
 {
 t0 = (y0 - ly0) / vy;
 t1 = (y1 - ly1) / vy;
 }

 bool t0_valid = t0 >= 0 && t0 <= 1;
 bool t1_valid = t1 >= 0 && t1 <= 1;
 if (t0_valid && t1_valid)
 {
 if (t0 <= t1)
 {
 RTCUS_ASSERT(x0 * x0 + y0 * y0 == r_sq);
 return sqrt((x0 - lx0) * (x0 - lx0) + (y0 - ly0) * (y0 - ly0));
 }
 else
 {
 RTCUS_ASSERT(x1 * x1 + y1 * y1 == r_sq);
 return sqrt((x1 - lx0) * (x1 - lx0) + (y1 - ly0) * (y1 - ly0));
 }
 }
 else if (t0_valid)
 {
 RTCUS_ASSERT(x0 * x0 + y0 * y0 == r_sq);
 return sqrt((x0 - lx0) * (x0 - lx0) + (y0 - ly0) * (y0 - ly0));
 }
 else if (t1_valid)
 {
 RTCUS_ASSERT(x1 * x1 + y1 * y1 == r_sq);
 return sqrt((x1 - lx0) * (x1 - lx0) + (y1 - ly0) * (y1 - ly0));
 }
 else
 return this->max_distance_;

 }
 */
//FINE LINE COLLISION CHECKING
/*
 double discriminant = ay * y_c + vx_pw2 * yc_sq + ax * x_c + axy * x_c * y_c + ax2 * xc_sq;

 if (discriminant >= 0)
 {
 //THERE IS COLLISION
 double base = -vx * x0 + vx * x_c + vy * y0 - vy * y_c;
 double t0 = (base + sqrt(discriminant)) / denominator;
 //double t1 = (base + sqrt(discriminant)) / denominator;
 if (t0 < 0 || t0 > 1)
 return this->max_distance_;

 double collision_x = x0 + vx * t0;
 double collision_y = y0 + vy * t0;
 double distance = sqrt(pow(x0 - collision_x, 2) + pow(y0 - collision_y, 2));
 return distance;
 }
 else
 return this->max_distance_;*/
/*
 }
 */
template<typename ObstacleVectorLike>
  inline bool LinearTrajectoryClearance::computeClerance_aux(const TwoPointSegment& action,
                                                             const ObstacleVectorLike& obstacles,
                                                             const ICircularRobotShape& shape,
                                                             DefaultClearanceCost& clearance) const
  {
    //double r, r_sq, vx, vy, vxy, vx_pw2, vy_pw2, bounding_xmin, bounding_xmax, bounding_ymin, bounding_ymax, x0, y0,
    //       x0_sq, x1_sq, y0_sq, y1_sq, denominator, discriminant_base, ax, ay, axy, ax2;
    //this->getfixedVariables(action, shape, r, r_sq, vx, vy, vxy, vx_pw2, vy_pw2, bounding_xmin, bounding_xmax,
    //                           bounding_ymin, bounding_ymax, x0, y0, x0_sq, x1_sq, y0_sq, y1_sq, denominator,
    //                           discriminant_base, ax, ay, axy, ax2);

    double r, r_sq, vx, vy, bounding_xmin, bounding_xmax, bounding_ymin, bounding_ymax, xa, ya, xb, yb, D, D_sq, dr,
           dr_sq;
    this->getfixedVariables(action, shape, r, r_sq, vx, vy, bounding_xmin, bounding_xmax, bounding_ymin, bounding_ymax,
                            xa, ya, xb, yb, D, D_sq, dr, dr_sq);

    clearance.linear_collision_distance_ = this->max_distance_;
    for (unsigned int i = 0; i < obstacles.size(); i++)
    {
      const PointXY& o = obstacles[i];
      //double distance = this->computeDistance(action, shape, o, r, r_sq, vx, vy, vxy, vx_pw2, vy_pw2, bounding_xmin,
      //                                       bounding_xmax, bounding_ymin, bounding_ymax, x0, y0, x0_sq, x1_sq, y0_sq,
      //                                       y1_sq, denominator, discriminant_base, ax, ay, axy, ax2);

      PointXY collision_point;
      double distance = this->computeDistance(action, shape, o, r, r_sq, vx, vy, bounding_xmin, bounding_xmax,
                                              bounding_ymin, bounding_ymax, xa, ya, xb, yb, D, D_sq, dr, dr_sq,
                                              collision_point);

      if (distance < clearance.linear_collision_distance_)
      {
        clearance.linear_collision_distance_ = distance;
        clearance.collision_point_ = collision_point;
      }
    }
    clearance.collision = (clearance.linear_collision_distance_ != this->max_distance_);
    return clearance.collision;

  }

bool LinearTrajectoryClearance::computeClearance(const TwoPointSegment& action, const vector<pcl::PointXY>& obstacles,
                                                 const ICircularRobotShape& shape,
                                                 DefaultClearanceCost& clearance) const
{
  return computeClerance_aux(action, obstacles, shape, clearance);
}

bool LinearTrajectoryClearance::computeClearance(const TwoPointSegment& action,
                                                 const pcl::PointCloud<pcl::PointXY>& obstacles,
                                                 const ICircularRobotShape& shape,
                                                 DefaultClearanceCost& clearance) const
{
  return this->computeClerance_aux(action, obstacles.points, shape, clearance);
}

bool LinearTrajectoryClearance::computeClearance(const TwoPointSegment& action, const pcl::PointXY& obstacle,
                                                 const ICircularRobotShape& shape,
                                                 DefaultClearanceCost& clearance) const
{
  double r, r_sq, vx, vy, bounding_xmin, bounding_xmax, bounding_ymin, bounding_ymax, xa, ya, xb, yb, D, D_sq, dr,
         dr_sq;
  this->getfixedVariables(action, shape, r, r_sq, vx, vy, bounding_xmin, bounding_xmax, bounding_ymin, bounding_ymax,
                          xa, ya, xb, yb, D, D_sq, dr, dr_sq);
  double distance = this->computeDistance(action, shape, obstacle, r, r_sq, vx, vy, bounding_xmin, bounding_xmax,
                                          bounding_ymin, bounding_ymax, xa, ya, xb, yb, D, D_sq, dr, dr_sq,
                                          clearance.collision_point_);

  return distance != this->max_distance_;
}

}
}

