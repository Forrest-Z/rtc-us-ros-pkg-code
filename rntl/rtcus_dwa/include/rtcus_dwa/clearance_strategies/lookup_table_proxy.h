/*
 * lookup_table_proxy.h
 *
 *  Created on: Feb 26, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef LOOKUP_TABLE_PROXY_H_
#define LOOKUP_TABLE_PROXY_H_

#include <rtcus_navigation/trajectory_clearance/trajectory_clearance.h>
#include <rtcus_navigation/trajectory_clearance/trajectory_clearance_lookup_table_decorator.h>
#include <rtcus_robot_shapes/polygonal_robot.h>
#include <rtcus_dwa/dwa_command_cost.h>
#include <rtcus_dwa/dwa_config.h>
#include <rtcus_navigation/trajectory_clearance/covariant_trajectory_clearance.h>

#include <math.h>

namespace rtcus_dwa
{
using namespace rtcus_navigation::trajectory_clearance;
using namespace boost;
using namespace rtcus_nav_msgs;
using namespace rtcus_robot_shapes;
using namespace boost;
using namespace std;
using namespace rtcus_dwa;
using namespace rtcus_navigation;

class LookupConfig
{

public:
  typedef TrajectoryClearanceLookupTable2D<Twist2D, PolygonalRobot> TLookupTable;
  typedef TrajectoryClearance<Twist2D, pcl::PointXY, PolygonalRobot, DefaultClearanceCost> TLookupWrapped;

  shared_ptr<TLookupTable> lookup_table_;

  LookupConfig() :
      allow_slow_computations_(true), workspace_longitude_(40), resolution_(500), action_count_(400)
  {
    ros::NodeHandle nh("~planner/lookup_table");
    if (!nh.getParam("lookup_area_longitude", this->workspace_longitude_))
      nh.setParam("lookup_area_longitude", this->workspace_longitude_);

    if (!nh.getParam("use_computation_outside_obstacles", this->allow_slow_computations_))
      nh.setParam("use_computation_outside_obstacles", this->allow_slow_computations_);

    int tmp;
    if (nh.getParam("action_count", tmp))
    {
      this->action_count_ = tmp;
      RTCUS_ASSERT_MSG(this->action_count_>1, "Incorrect Lookup table parameter - number of actions %d", tmp);
    }
    else
      nh.setParam("action_count", (int)this->action_count_);

    if (nh.getParam("resolution", tmp))
    {
      tmp = this->resolution_;
      RTCUS_ASSERT_MSG(this->resolution_>1, "Incorrect Lookup table parameter - resolution_ %d", tmp);
    }
    else
      nh.setParam("resolution", (int)this->resolution_);

    knorm_ = (float)(action_count_) / M_PI_2;
  }

  //it can be used as get_index_from_action, but it would be computationally less efficent since a lot of atans shouldd be computed (one for each obstacle)
  void update_and_compute_current_action_index(const CommandCost<Twist2D>& action)
  {
    static const double NUMERICAL_MIN = 0.0002;
    int kurvature_index;
    if (fabs(action.getAction().angular) < NUMERICAL_MIN)
    {
      kurvature_index = action_count_ - 1;
    }
    else
    {
      //double alpha = atan2(action.linear, action.angular);
      double alpha = action.getNormalizedIndexationFactor();
      //ROS_INFO("alpha: %f", alpha);
      if (alpha < 0 || alpha > M_PI)
      {
        //we do not accept going backwards
        //alpha += 2 * M_PI;
        RTCUS_ASSERT_MSG(
            alpha > 0,
            "Lookup table, alpha action space for kurvature (%lf) Only linear positive velocities are allowed for this implementation", alpha);
      }
      //ROS_INFO("alpha: %f", alpha);
      if (alpha > M_PI_2 && alpha < M_PI)
      {
        alpha = M_PI - alpha;
        //ROS_INFO("alpha: %f", alpha);
        kurvature_index = -((int)this->action_count_) + 1
            + std::min(std::max(knorm_ * alpha, 0.0), this->action_count_ - 1.0);
      }
      else
      {
        kurvature_index = std::min(std::max(knorm_ * alpha, 0.0), this->action_count_ - 1.0);
      }
    }
    //ROS_INFO("kurvature index %d", kurvature_index);
    current_action_index_ = kurvature_index;
  }

  void precomputeLookupTableFromShape(const PolygonalRobot& shape, const DwaConfig& config)
  {
    if (config.use_clearance_lookup_table
        && (this->lookup_table_ == NULL || (this->lookup_table_ != NULL && !this->lookup_table_->precomputed())))
    {
      ROS_INFO("Request for lookup table. Does it exist? -> %d", (bool)this->lookup_table_);
      if (!lookup_table_)
      {
        ROS_WARN("Recomputing clearance lookup table because of robot shape changed.");
        ROS_INFO("Clearance Strategy Cost. Creating Lookup table");
        if (this->clerance_method_)
          this->create(clerance_method_);
        else
          ROS_ERROR("The method to do the lookup table has not been specified");
      }

      //disable temporally distance filtering for generate lookup for any distance
      clerance_method_->setMaxDistance(numeric_limits<double>::max());
      lookup_table_->precomputeLookupTable(shape);
      clerance_method_->setMaxDistance(config.max_collision_distance);

    }
  }
  template<typename TSrc>
    void setFromType(shared_ptr<TSrc> clearance_strategy)
    {
      this->clerance_method_ = ExplicitCovarianceTraits<TSrc, TLookupWrapped>::CovariantDecorator(clearance_strategy);

      RTCUS_ASSERT_MSG(this->clerance_method_, "Incorrect clearance method for compute the lookup table.");
    }

protected:

  void create(shared_ptr<TLookupWrapped> clearance_strategy)
  {
    lookup_table_ = shared_ptr<TLookupTable>(
        new TLookupTable(clearance_strategy, this->workspace_longitude_, this->resolution_, this->action_count_,
                         boost::bind(&LookupConfig::get_index_from_action, this, _1),
                         boost::bind(&LookupConfig::get_action_from_index, this, _1, _2),
                         this->allow_slow_computations_));
  }

  shared_ptr<TLookupWrapped> clerance_method_;
  rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig kinodynamic_desc;
  bool allow_slow_computations_;
  double workspace_longitude_;
  unsigned int resolution_;
  unsigned int action_count_;
  float knorm_;
  int current_action_index_;

  //this is calles by the lookup table for each obstacle. currently used a Cache.
  int get_index_from_action(const Twist2D& action) const
  {
    return current_action_index_;
  }

  //this is just for the generation of the lookup table so performance ins not too much important
  void get_action_from_index(unsigned int action_index, Twist2D& action) const
  {
    float alpha = (M_PI_2 / (action_count_)) * action_index;
    action.linear = sin(alpha);
    action.angular = cos(alpha);
  }
};
}

#endif /* LOOKUP_TABLE_PROXY_H_ */
