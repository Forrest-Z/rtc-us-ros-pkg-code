/*
 * trajectory_clearance_lookup_table_decorator.h
 *
 *  Created on: Dec 11, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef TRAJECTORY_CLEARANCE_LOOKUP_TABLE_DECORATOR_H_
#define TRAJECTORY_CLEARANCE_LOOKUP_TABLE_DECORATOR_H_

#include <rtcus_navigation/trajectory_clearance/trajectory_clearance.h>
#include <rtcus_navigation/trajectory_clearance/default_clearance_cost.h>
#include <boost/foreach.hpp>
#include <vector>
#include <rtcus_assert/rtcus_assert.h>
#include <pcl/point_cloud.h>
#include <list>

namespace rtcus_navigation
{
namespace trajectory_clearance
{
using namespace rtcus_navigation::trajectory_clearance;
/**
 * \brief Lookup tables to compute the clearance or distance to obstacles given a trajectory are very useful in the
 * obstacle avoidance field. They have been used and studied intensively in
 *
 * They have been used for circular trajectories in:
 * Originally proposed by;
 * - Schlegel, C. 2004. “Navigation and Execution for Mobile Robots in Dynamic Environments: An Integrated Approach.”
 * - Minguez, Javier. 2002. “Robot Shape, Kinematics, and Dynamics in Sensor-Based Motion Planning”.
 *
 * But also is it possible to generalize this to non circular trajectories:
 * - Luis, Jose, and Blanco Claraco. 2009. “Contributions to Localization, Mapping and Navigation in Mobile Robotics.” Robotics.
 * - Howard, T. M., and a. Kelly. 2007. “Optimal Rough Terrain Trajectory Generation for Wheeled Mobile Robots.” The International Journal of Robotics Research 26 (2) (February 1): 141–166. doi:10.1177/0278364906075328. http://ijr.sagepub.com/cgi/doi/10.1177/0278364906075328.
 * */
template<typename ActionType, typename RobotShapeType>
  class TrajectoryClearanceLookupTable2D : public TrajectoryClearance<ActionType, pcl::PointXY, RobotShapeType,
                                               DefaultClearanceCost>,
                                           public TrajectoryClearance<ActionType, pcl::PointCloud<pcl::PointXY>,
                                               RobotShapeType, DefaultClearanceCost>,
                                           public TrajectoryClearance<ActionType, vector<pcl::PointXY>, RobotShapeType,
                                               DefaultClearanceCost>
  {
  protected:

    boost::shared_ptr<TrajectoryClearance<ActionType, pcl::PointXY, RobotShapeType, DefaultClearanceCost> > decorated_clearance_method_;
    typedef std::list<std::pair<unsigned int, DefaultClearanceCost> > CellClearanceInfo;
    typedef const CellClearanceInfo ConstCellClearanceInfo;

    //3D lookup table. width * height * actions -> clearance(obst,action)=lookup_table_(obst.x,obst.y,action)
    std::vector<std::vector<CellClearanceInfo> > lookup_table_;

    float distance_;
    unsigned int resolution_;
    unsigned int action_count_;
    unsigned int origin_pixel_x_;
    unsigned int origin_pixel_y_;
    bool allow_slow_computations_;
    //helpers variables
    float pixel_distance_;
    pcl::PointXY origin_;
    float k_;
    bool precomputed_;
    boost::function<int(const ActionType&)> get_motion_primitive_index_;
    boost::function<void(int, ActionType&)> get_action_from_index_;

  public:
    TrajectoryClearanceLookupTable2D(
        boost::shared_ptr<TrajectoryClearance<ActionType, pcl::PointXY, RobotShapeType, DefaultClearanceCost> > decorated_clearance_method,
        float distance, unsigned int resolution, unsigned int action_count,
        boost::function<int(const ActionType&)> get_index_from_action,
        boost::function<void(unsigned int, ActionType&)> get_action_from_index, bool allow_slow_computations = false) :
        decorated_clearance_method_(decorated_clearance_method), lookup_table_(), distance_(distance), resolution_(
            resolution), action_count_(action_count), allow_slow_computations_(allow_slow_computations), precomputed_(
            false), get_motion_primitive_index_(get_index_from_action), get_action_from_index_(get_action_from_index)
    {

      RTCUS_ASSERT(decorated_clearance_method_);
      RTCUS_ASSERT(resolution_ >= 3);
      RTCUS_ASSERT(action_count_ >= 1);
      //to avoid numerical instabilities
      RTCUS_ASSERT(distance_ > 0.002);

      //assert odd size to have an exact center
      if (resolution_ % 2 == 0)
        resolution_ += 1;

      origin_pixel_x_ = 1 + resolution_ / 2;
      origin_pixel_y_ = 1 + resolution_ / 2;

      origin_.x = distance_ / 2.0;
      origin_.y = distance_ / 2.0;
      this->k_ = (float)resolution_ / distance_;

      pixel_distance_ = distance_ / resolution_;

      initialize_lookup_table();
    }
    virtual ~TrajectoryClearanceLookupTable2D()
    {
    }

    virtual void setMaxDistance(double distance)
    {
      this->decorated_clearance_method_->setMaxDistance(distance);
    }
    virtual double getMaxDistance() const
    {
      return this->decorated_clearance_method_->getMaxDistance();
    }

    boost::shared_ptr<TrajectoryClearance<ActionType, pcl::PointXY, RobotShapeType, DefaultClearanceCost> > getDecoratedClearanceMethod()
    {
      return decorated_clearance_method_;
    }

    inline bool precomputed()
    {
      return precomputed_;
    }

    void precomputeLookupTable(const RobotShapeType& shape)
    {

      for (unsigned int action_index = 0; action_index < this->action_count_; action_index++)
      {
        ROS_INFO(" * Precomputing Lookup table: %lf percent", 100.0 * (float)action_index / (float)this->action_count_);
        precomputeAction(action_index, shape);
      }
      this->precomputed_ = true;
    }

    void setIndexFromActionFunction(const boost::function<int(const ActionType&)>& fn)
    {
      this->get_motion_primitive_index_ = fn;
    }

    void setActionFromIndex(const boost::function<void(int, ActionType&)>& fn)
    {
      this->get_action_from_index_ = fn;
    }

    bool checkAcctionIndexMapping() const
    {
      int error = 0;
      for (int i = 0; i < (int)action_count_; i++)
      {
        ActionType action;
        this->get_action_from_index_(i, action);
        int motprim_index = this->get_motion_primitive_index_(action);
        error = abs(i - motprim_index);
        printf("Error: %d - \n", error);
        if (error > 0)
        {
          std::stringstream ss;
          ss << "action index got from action: " << motprim_index << " (while it should be - " << i << " <<action"
              << std::endl << " ERROR for action:" << std::endl << action << std::endl;
          printf("%s", ss.str().c_str());
          error = true;
        }
      }
      return !error;
    }

    virtual bool computeClearance(const ActionType& action, const pcl::PointXY& obstacle, const RobotShapeType& shape,
                                  DefaultClearanceCost& clearance)
    {
      return ((const TrajectoryClearanceLookupTable2D&)*this).computeClearance(action, obstacle, shape, clearance);
    }

    virtual bool computeClearance(const ActionType& action, const vector<pcl::PointXY>& obstacles,
                                  const RobotShapeType& shape, DefaultClearanceCost& clearance) const
    {

      return this->computeClearance_auxit(obstacles.begin(), obstacles.end(), action, shape, clearance);
    }

    virtual bool computeClearance(const ActionType& action, const pcl::PointCloud<pcl::PointXY>& obstacles,
                                  const RobotShapeType& shape, DefaultClearanceCost& clearance) const
    {
      return this->computeClearance_auxit(obstacles.points.begin(), obstacles.points.end(), action, shape, clearance);
    }

    inline void correct_symetric_obstacle(unsigned int& ox, unsigned int& oy) const
    {
      oy = resolution_ - oy - 1;
    }

    inline unsigned int computeActionIndex(const ActionType& action, bool& symetry_action_got) const
    {
      int action_index = this->get_motion_primitive_index_(action);
      if (action_index < 0)
      {
        //symetric:
        action_index = this->action_count_ + action_index - 1;
        symetry_action_got = true;
      }
      else
        symetry_action_got = false;

      //ROS_INFO("action index got: %d. Obstacle index [%d,%d]", action_index, ox, oy);
      RTCUS_ASSERT_MSG(
          action_index > -((int)this->action_count_) && action_index < (int)this->action_count_,
          "The action index is not mapped in the lookup table. It must belong to the interval 0 - %d while given it was %d",
          this->action_count_, action_index);

      return action_index;
    }

    virtual bool computeClearance(const ActionType& action, const pcl::PointXY& obstacle, const RobotShapeType& shape,
                                  DefaultClearanceCost& clearance) const
    {
      unsigned int ox;
      unsigned int oy;
      if (!this->getObstacleIndexFromReal(obstacle, ox, oy))
      {
        ROS_WARN_THROTTLE(1.0, "Obstacle outside the lookup_window (%d, %d)-> (%lf,%lf)", ox, oy, obstacle.x,
                          obstacle.y);
        if (allow_slow_computations_)
        {
          return this->decorated_clearance_method_->computeClearance(action, obstacle, shape, clearance);
        }
        else
        {
          RTCUS_ASSERT_MSG(
              false,
              "this obstacle is outside of the lookup table. Clearance evaluation crashed. If you wan't allow outside clearance computations, set the parameter allow_slow_computations=true");
          clearance.linear_collision_distance_ = std::numeric_limits<double>::quiet_NaN();
          return false;
        }
      }
      else
      {
        bool symetric;
        const DefaultClearanceCost* clearance_ret;
        unsigned int action_index = computeActionIndex(action, symetric);
        if (symetric)
          this->correct_symetric_obstacle(ox, oy);
        ConstCellClearanceInfo& current_cell = this->lookup_table_[ox][oy];
        clearance_ret = this->find(current_cell, action_index);
        return clearance_ret != NULL && clearance_ret->linear_collision_distance_ != std::numeric_limits<double>::max();
      }
    }

  private:

    /**
     * \return false if there is an outbound access to the lookup-table
     * */
    inline bool getObstacleIndexFromReal(const pcl::PointXY& obstacle, unsigned int& ox, unsigned int& oy) const
    {
      int oxa = this->k_ * (obstacle.x + origin_.x);
      int oya = this->k_ * (obstacle.y + origin_.y);
      bool out_of_bounds = oxa < 0 || oxa >= (int)resolution_ || oya < 0 || oya >= (int)resolution_;
      ox = oxa;
      oy = oya;
      return !out_of_bounds;
    }

    inline bool getObstacleRealFromIndex(unsigned int ox, unsigned int oy, pcl::PointXY& obstacle_position) const
    {
      obstacle_position.x = ox * pixel_distance_ - origin_.x;
      obstacle_position.y = oy * pixel_distance_ - origin_.y;
      return true;
    }

    void initialize_lookup_table()
    {
      //ROS_DEBUG("Reserving space lookup table: resolution %d, action count %d", resolution_, action_count_);
      //lookup_table_.resize(action_count_);
      //lookup_table_=new float**[resolution_];
#pragma omp parallel for
      for (unsigned int k = 0; k < this->action_count_; k++)
      {
        ROS_DEBUG("resizing action %d to cols %d", k, resolution_);
        lookup_table_.resize(resolution_);
        //lookup_table_[ox]=new float*[resolution_];
        for (unsigned int ox = 0; ox < resolution_; ox++)
        {
          ROS_DEBUG("\nresizing col %d of action %d to %d", ox, k, resolution_);
          lookup_table_[ox].resize(resolution_);

          for (unsigned int oy = 0; oy < resolution_; oy++)
            lookup_table_[ox][oy] = std::list<std::pair<unsigned int, DefaultClearanceCost> >();
        }
      }
    }

  private:
    inline const rtcus_navigation::trajectory_clearance::DefaultClearanceCost* find(ConstCellClearanceInfo& map_cell,
                                                                                    unsigned int action_index) const
    {
      rtcus_navigation::trajectory_clearance::DefaultClearanceCost* result = NULL;
      for (ConstCellClearanceInfo::const_iterator it = map_cell.begin(); it != map_cell.end(); it++)
        if (it->first == action_index)
          return &(it->second);

      return result;
    }

    template<typename Titerator>
      inline bool computeClearance_auxit(Titerator itbegin, Titerator itend, const ActionType& action,
                                         const RobotShapeType& shape, DefaultClearanceCost& computed_clearance) const
      {
        computed_clearance.linear_collision_distance_ = std::numeric_limits<double>::max();
        bool symetric;
        unsigned int action_index = computeActionIndex(action, symetric);
        for (Titerator it = itbegin; it != itend; it++)
        {
          bool exist_collision = false;
          unsigned int ox;
          unsigned int oy;
          const DefaultClearanceCost* clearance_candidate;
          DefaultClearanceCost tmp;

          if (!this->getObstacleIndexFromReal(*it, ox, oy))
          {
            ROS_WARN_THROTTLE(1.0, "Obstacle outside the lookup_window (%d, %d)-> (%lf,%lf)", ox, oy, it->x, it->y);

            if (allow_slow_computations_)
            {

              exist_collision = this->decorated_clearance_method_->computeClearance(action, *it, shape, tmp);
              clearance_candidate = &tmp;

            }
            else
            {
              RTCUS_ASSERT_MSG(
                  false,
                  "this obstacle is outside of the lookup table. Clearance evaluation crashed. If you wan't allow outside clearance computations, set the parameter allow_slow_computations=true");
            }
          }
          else
          {
            if (symetric)
              this->correct_symetric_obstacle(ox, oy);

            ConstCellClearanceInfo& current_cell = this->lookup_table_[ox][oy];
            clearance_candidate = this->find(current_cell, action_index);
            exist_collision = (clearance_candidate != NULL
                && clearance_candidate->linear_collision_distance_ != std::numeric_limits<double>::max());
          }

          //current_collision = this->computeClearance(action, *it, shape, lookup_table_entry);
          if (exist_collision
              && clearance_candidate->linear_collision_distance_ < computed_clearance.linear_collision_distance_)
            computed_clearance = *clearance_candidate;

        }
        return computed_clearance.linear_collision_distance_ != std::numeric_limits<double>::max();
      }

    void precomputeAction(unsigned int action_index, const RobotShapeType& robot_shape)
    {
      ActionType action;
      this->get_action_from_index_(action_index, action);

#pragma omp parallel for
      for (unsigned int ox = 0; ox < resolution_; ox++)
        for (unsigned int oy = 0; oy < resolution_; oy++)
        {
          pcl::PointXY obstacle_position;
          this->getObstacleRealFromIndex(ox, oy, obstacle_position);

          DefaultClearanceCost clearance;
          bool collision = this->decorated_clearance_method_->computeClearance(action, obstacle_position, robot_shape,
                                                                               clearance);
          //CACHE THE DATA
          if (collision)
          {
            if (clearance.linear_collision_distance_ == 0)
              ROS_WARN_THROTTLE(
                  0.2,
                  "¿Is it Correct the internal clearance algorithm? Collision detected, but clearance =0 [kindex %d ox %d oy %d]: %lf",
                  action_index, ox, oy, clearance.linear_collision_distance_);

            RTCUS_ASSERT(
                clearance.collision && clearance.linear_collision_distance_ == clearance.linear_collision_distance_);
            std::pair<unsigned int, DefaultClearanceCost> entry;
            entry.first = action_index;
            entry.second = clearance;
            this->lookup_table_[ox][oy].push_back(entry);
          }

        }
    }
  };
}
}

#endif /* TRAJECTORY_CLEARANCE_LOOKUP_TABLE_DECORATOR_H_ */
