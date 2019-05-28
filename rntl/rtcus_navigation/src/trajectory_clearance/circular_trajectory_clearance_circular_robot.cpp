/*
 * circular_trajectory_clearance_circular_robot.cpp
 *
 *  Created on: Dec 10, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_navigation/trajectory_clearance/circular_trajectory_clearance_circular_robot.h>
#include <pcl/point_types.h>
#include <limits>
#include <rtcus_assert/rtcus_assert.h>
#include <opencv2/opencv.hpp>

namespace rtcus_navigation
{
namespace trajectory_clearance
{
using namespace rtcus_robot_shapes::interfaces;
using namespace rtcus_nav_msgs;
using namespace pcl;
using namespace std;

/**
 * \brief vectorial trajectory collision checking for a circular trajectory of a circular robot with an obstacle point
 * */

CircularTrajectoryClearanceForCircularRobot::~CircularTrajectoryClearanceForCircularRobot()
{
}

CircularTrajectoryClearanceForCircularRobot::CircularTrajectoryClearanceForCircularRobot()
{
  second_distance_ = std::numeric_limits<t_float>::max();
  //nearest_collision_point;
  //furtherst_collision_point;
  max_distance_ = std::numeric_limits<t_float>::max();
  forward_ = true;
  compute_second_point_ = false;
  this->obstacle_inflation_ = 0;
}

double CircularTrajectoryClearanceForCircularRobot::getMaxDistance() const
{
  return max_distance_;
}

void CircularTrajectoryClearanceForCircularRobot::setMaxDistance(double distance)
{
  max_distance_ = distance;
}

CircularTrajectoryClearanceForCircularRobot::ClearanceResult CircularTrajectoryClearanceForCircularRobot::computeClearance(
    const Twist2D& action, const pcl::PointXY& obstacle, double robot_radius, double max_collision_distance,
    bool forward, t_float &distance, t_float& second_distance, pcl::PointXY & nearest_collision_point,
    pcl::PointXY& furtherst_collision_point, t_float obstacle_inflation) const
{
  return computeClearance_auxb(action, obstacle, robot_radius, max_collision_distance, forward, distance,
                               second_distance, nearest_collision_point, furtherst_collision_point, obstacle_inflation);

}

bool CircularTrajectoryClearanceForCircularRobot::computeClearance(const Twist2D& action,
                                                                   const pcl::PointCloud<pcl::PointXY>& obstacles,
                                                                   const ICircularRobotShape& shape,
                                                                   DefaultClearanceCost& distance) const
{
  return const_cast<CircularTrajectoryClearanceForCircularRobot*>(this)->computeClearance_auxit(
      obstacles.points.begin(), obstacles.points.end(), action, shape, distance);
}

bool CircularTrajectoryClearanceForCircularRobot::computeClearance(const Twist2D& action,
                                                                   const std::vector<pcl::PointXY>& obstacles,
                                                                   const ICircularRobotShape& shape,
                                                                   DefaultClearanceCost& distance) const
{
  return const_cast<CircularTrajectoryClearanceForCircularRobot*>(this)->computeClearance_auxit(obstacles.begin(),
                                                                                                obstacles.end(), action,
                                                                                                shape, distance);
}

bool CircularTrajectoryClearanceForCircularRobot::computeClearance(const Twist2D& action, const pcl::PointXY& obstacle,
                                                                   const ICircularRobotShape& shape,
                                                                   DefaultClearanceCost& distance)
{
  return this->computeClearance_aux(action, obstacle, shape, distance);
}

bool CircularTrajectoryClearanceForCircularRobot::computeClearance(const Twist2D& action, const pcl::PointXY& obstacle,
                                                                   const ICircularRobotShape& shape,
                                                                   DefaultClearanceCost& distance) const
{
  return const_cast<CircularTrajectoryClearanceForCircularRobot*>(this)->computeClearance_aux(action, obstacle, shape,
                                                                                              distance);
}

//--------------------------------------------------------------------------------------------------------------------------------
template<typename t_float>
  inline void compute_collision_distance(t_float ICR_y, t_float x, t_float y, t_float r, t_float& distance,
                                         t_float& angle)
  {
    //DO NOT CHANGE THE VARIABLE ORDER, IT LOOKS WRONG BUT IT IS OK
    angle = atan2(x /*- ICR_x*/, ICR_y - y);
    if (angle < 0.0)
      angle += 2.0 * M_PI;
    distance = angle * r;
  }

template<typename Titerator>
  inline bool CircularTrajectoryClearanceForCircularRobot::computeClearance_auxit(Titerator itObstacleStart,
                                                                                  Titerator itObstaclesEnd,
                                                                                  const Twist2D& action,
                                                                                  const ICircularRobotShape& shape,
                                                                                  DefaultClearanceCost& clearance)
  {
    clearance.linear_collision_distance_ = std::numeric_limits<double>::max();
    t_float max_distance = this->max_distance_;
    for (Titerator obstacle = itObstacleStart; obstacle != itObstaclesEnd; obstacle++)
    {
      double obstacle_path_dist;
      const pcl::PointXY& o = *obstacle;
      CircularTrajectoryClearanceForCircularRobot::ClearanceResult raytraceResult = this->computeClearance(
          action, o, shape.getRadius(), max_distance, this->forward_, obstacle_path_dist, this->second_distance_,
          this->nearest_collision_point_, this->furtherst_collision_point_, this->obstacle_inflation_);

      if (raytraceResult != NO_COLLISION_FOUND && obstacle_path_dist < max_distance)
      {
        clearance.linear_collision_distance_ = obstacle_path_dist;
        clearance.collision_point_=nearest_collision_point_;
        max_distance = obstacle_path_dist;
        if (raytraceResult == INSIDE_OBSTACLE)
          break;
      }
    }
    bool collision = (clearance.linear_collision_distance_ != std::numeric_limits<double>::max());
    if (!collision)
      clearance.linear_collision_distance_ = std::numeric_limits<double>::quiet_NaN();

    clearance.collision = collision;
    return collision;
  }

inline bool CircularTrajectoryClearanceForCircularRobot::computeClearance_aux(const Twist2D& action,
                                                                              const pcl::PointXY& obstacle,
                                                                              const ICircularRobotShape& shape,
                                                                              DefaultClearanceCost& distance)
{
  this->result_ = this->computeClearance(action, obstacle, shape.getRadius(), this->max_distance_, this->forward_,
                                         distance.linear_collision_distance_, this->second_distance_,
                                         this->nearest_collision_point_, this->furtherst_collision_point_,
                                         this->obstacle_inflation_);

  distance.collision_point_ = nearest_collision_point_;
  distance.collision = this->result_ != NO_COLLISION_FOUND;

  return (this->result_ != NO_COLLISION_FOUND);
}
inline CircularTrajectoryClearanceForCircularRobot::ClearanceResult CircularTrajectoryClearanceForCircularRobot::computeClearance_auxb(
    const Twist2D& action, const pcl::PointXY& obstacle, double robot_radius, double max_collision_distance,
    bool forward, t_float &distance, t_float& second_distance, pcl::PointXY & nearest_collision_point,
    pcl::PointXY& furtherst_collision_point, t_float obstacle_inflation) const
{

  //POSTCONDITION
  /*
   RTCUS_ASSERT_MSG(obstacle_info.near_collision_distance >= 0 && obstacle_info.far_collision_distance >= 0,
   "obstacle obstacle.x %f obstacle.y %f || action.linear %lf action.angular %lf --- first distance (%lf) second distance (%lf)",
   obstacle.x, obstacle.y, (double)action.linear, (double)action.angular, obstacle_info.near_collision_distance,
   obstacle_info.far_collision_distance);
   */

//CIRCULAR TRAJECTORIES OR LINEAR TRAJECTORIES?
  //COMPUTE COLLISIONS
  //we have to find the intersection of the two circles (trajectory, and obstacles)
  //http://paulbourke.net/geometry/2circle/
  // BE CAREFUL: Numerical inestabilities getting the rotation radious
  if (fabs(action.angular) > MIN_DIVISION_DENOMINATOR)
  {
    t_float r = action.linear / action.angular;
    t_float ICR_y = r;
//take obstacle center
    //take middle point
    t_float vx = obstacle.x; // - ICR_x ==0
    t_float vy = obstacle.y - ICR_y;

    t_float d_squared = vx * vx + vy * vy;
    t_float inflated_robot_radius = robot_radius + obstacle_inflation;
    t_float squared_inflated_radius;
    r = fabs(r);

    //BROAD COLLISION CHECK
    if (d_squared > (r + inflated_robot_radius) * (r + inflated_robot_radius))
    {
      //Obstacle too far -> NO COLLISION
      distance = max_collision_distance;
      second_distance = max_collision_distance;
      return NO_COLLISION_FOUND;
    }
    //If d < |r0 - r1| then there are no solutions because one circle is contained within the other.
    else if (d_squared < (r - inflated_robot_radius) * (r - inflated_robot_radius))
    {
      //only the small circle can be contained
      if (obstacle.x * obstacle.x + obstacle.y * obstacle.y < inflated_robot_radius * inflated_robot_radius)
      {
        distance = 0;
        return INSIDE_OBSTACLE;
      }
      else if (d_squared > inflated_robot_radius) //then trajectory surrounds obstacles
      {
        distance = max_collision_distance;
        second_distance = max_collision_distance;
        return NO_COLLISION_FOUND;
      }
      else //then the trajectory is inside the obstacle
      {
        distance = max_collision_distance;
        second_distance = max_collision_distance;
        return INTERNAL_LOOP;
      }
    }
    else if (obstacle.x * obstacle.x + obstacle.y * obstacle.y
        < (squared_inflated_radius = (inflated_robot_radius * inflated_robot_radius)))
    {
      distance = 0;
      return INSIDE_OBSTACLE;
    }
    else
    {
      t_float d = sqrt(d_squared);
      //Using d = a + b we can solve for a,
      t_float r_squared = r * r;
      t_float vxd = vx / d;
      t_float vyd = vy / d;

      t_float a = (r_squared - squared_inflated_radius + d_squared) / (2 * d);
      t_float median_point_x = a * vxd;
      t_float median_point_y = ICR_y + a * vyd;

      t_float discriminant = r_squared - a * a;
      RTCUS_ASSERT_MSG(
          discriminant >= 0.0,
          "Possible numerical instability computing the distance-> sqrt(d) where d= %lf -> action.linear %lf action.angular %lf obstacle.x %lf obstacle.y %lf obstacle radious %lf",
          discriminant, action.linear, action.angular, obstacle.x, obstacle.y, inflated_robot_radius);

      t_float h = sqrt(discriminant);

      //perpendicular vectors to action.linear
      t_float collision_x_1 = median_point_x + h * vyd;
      t_float collision_y_1 = median_point_y - h * vxd;

      t_float collision_x_2 = median_point_x - h * vyd;
      t_float collision_y_2 = median_point_y + h * vxd;

      //TURNING RIGHT -> FLIP AXIS Y TO CALCULATE DISTANCE
      bool flip = action.angular < 0;
      if (flip)
      {
        collision_y_1 = -collision_y_1;
        collision_y_2 = -collision_y_2;
        ICR_y = -ICR_y;
      }

      t_float collision_angle_1, collision_angle_2, distance_1, distance_2;
      t_float cross_product_to_chose = collision_x_1 * collision_y_2 - collision_y_1 * collision_x_2;

      if (forward)
      {
        if (cross_product_to_chose > 0.0)
        //if (collision_angle_1 < collision_angle_2)
        {
          compute_collision_distance<t_float>(ICR_y, collision_x_1, collision_y_1, r, distance_1, collision_angle_1);
          distance = distance_1;
          nearest_collision_point.x = collision_x_1;
          nearest_collision_point.y = collision_y_1;

          if (this->compute_second_point_)
          {
            compute_collision_distance<t_float>(ICR_y, collision_x_2, collision_y_2, r, distance_2, collision_angle_2);
            second_distance = distance_2;
            furtherst_collision_point.x = collision_x_2;
            furtherst_collision_point.y = collision_y_2;
          }
        }
        else
        {
          compute_collision_distance<t_float>(ICR_y, collision_x_2, collision_y_2, r, distance_2, collision_angle_2);
          distance = distance_2;
          nearest_collision_point.x = collision_x_2;
          nearest_collision_point.y = collision_y_2;

          if (this->compute_second_point_)
          {
            compute_collision_distance<t_float>(ICR_y, collision_x_1, collision_y_1, r, distance_1, collision_angle_1);
            second_distance = distance_1;
            furtherst_collision_point.x = collision_x_1;
            furtherst_collision_point.y = collision_y_1;
          }

        }

        //UNDO RIGHT TURNING FLIPPING
        if (flip)
        {
          nearest_collision_point.y = -nearest_collision_point.y;
          furtherst_collision_point.y = -furtherst_collision_point.y;
        }
      }
      else
        throw ros::Exception("Back speed NotImplemented...");

      // ROS_DEBUG(
      //     "Collision expected to o(%f,%f with radius %lf at %lf) (action.linear=%lf,action.angular=%lf) arc %lf distance ->%lf", obstacle.x, obstacle.y, inflated_robot_radius, sqrt(obstacle.x * obstacle.x + obstacle.y * obstacle.y), action.linear, action.angular, (distance * 180) / (r * M_PI), distance);
//and if te trajectory is inside?

      return COLLISION;
    }
  }
  else
  {
//ROS_INFO("avoiding numerical inestabilities");
//TODO: Solucionar esto que no funciona
//CALCULATE THE INTERSECTION
//ROS_INFO ("intersect line - circle"last_trajectory_markers_count_);
//as we are working in the car local frame
//check if collision
    t_float inflated_robot_radius = robot_radius + obstacle_inflation;
    if (fabs(obstacle.y) > inflated_robot_radius)
    {
      //ROS_INFO("Straigt line no collision obstacle.y=%f obst radious=%lf",obstacle.y,inflated_robot_radius);
      distance = max_collision_distance;
      second_distance = max_collision_distance;
      //ROS_INFO(
      //          "Collisin (straigth line)- no collision found A- to o(%f,%f with radious %lf at %lf) (action.linear=%lf,action.angular=%lf) distance ->%lf", obstacle.x, obstacle.y, inflated_robot_radius, sqrt(obstacle.x*obstacle.x+obstacle.y*obstacle.y), action.linear, action.angular, distance);

      return NO_COLLISION_FOUND;
    }

    else
    {
      //ROS_INFO("Straigt line collision obstacle(obstacle.x=%f,obstacle.y=%f)", obstacle.x, obstacle.y);
      //Sympy
      //In [11]: solve ((x-x_0)**2+y_0**2-r**2,x)
      //Out[11]: [x_0 + (r**2 - y_0**2)**(1/2), x_0 - (r**2 - y_0**2)**(1/2)]
      // (x,y) is the intersection point
      // constraint straigt trajectory, collision point in y_0== 0
      t_float discriminant = inflated_robot_radius * inflated_robot_radius - obstacle.y * obstacle.y;
      RTCUS_ASSERT_MSG(
          discriminant >= 0.0,
          "Posible numerical inestability computing the distance-> sqrt(d) where d= %lf -> action.linear %lf action.angular %lf",
          discriminant, action.linear, action.angular);
      t_float a = sqrt(discriminant);
      t_float x_col_0 = obstacle.x + a;
      t_float x_col_1 = obstacle.x - a;

      nearest_collision_point.y = 0;
      furtherst_collision_point.y = 0;
      if (forward)
      {
        if (x_col_0 < 0)
        {
          distance = x_col_1;
          second_distance = x_col_0;
        }
        else if (x_col_1 < 0)
        {
          distance = x_col_0;
          second_distance = x_col_1;
        }
        else if (x_col_0 > 0 && x_col_1 > 0)
        {
          distance = std::min(x_col_0, x_col_1);
          second_distance = std::max(x_col_0, x_col_1);
        }
        else
        {
          distance = std::min(x_col_0, x_col_1);
          second_distance = std::max(x_col_0, x_col_1);
          //ROS_INFO(
          //    "Collision (straigth line)- no collision found B- to o(%f,%f with radious %lf at %lf) (action.linear=%lf,action.angular=%lf) distance ->%lf", obstacle.x, obstacle.y, inflated_robot_radius, sqrt(obstacle.x*obstacle.x+obstacle.y*obstacle.y), action.linear, action.angular, distance);
        }

        if (distance < 0)
        {

          distance = max_collision_distance;
          second_distance = max_collision_distance;
          return NO_COLLISION_FOUND;
        }
        nearest_collision_point.x = distance;
        furtherst_collision_point.x = second_distance;

      }
      else //straight line and backward
      {
        throw ros::Exception("Back speed NotImplemented...");
      }

      t_float obs_dist = sqrt(obstacle.x * obstacle.x + obstacle.y * obstacle.y);
      if (obs_dist < inflated_robot_radius)
      {
        distance = 0;
        //ROS_INFO("straigt line collision with obstacle %lf %lf and radoius %lf", obstacle.x, obstacle.y, inflated_robot_radius);
        return INSIDE_OBSTACLE;
      }
      else
        return COLLISION;
    }
  }
}

}
}

