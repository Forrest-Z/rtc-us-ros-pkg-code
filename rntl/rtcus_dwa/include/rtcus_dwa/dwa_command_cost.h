/*
 * dwa_command__cost.h
 *
 *  Created on: Oct 8, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef DWA_command__COST_H_
#define DWA_command__COST_H_

#include <rtcus_nav_msgs/Twist2D.h>
#include <rtcus_nav_msgs/DynamicState2D.h>
#include <rtcus_dwa/common.h>
#include <rtcus_kinodynamic_description/NonHolonomicKinoDynamicsConfig.h>
#include <rtcus_navigation/trajectory_clearance/default_clearance_cost.h>

namespace rtcus_dwa
{
using namespace std;
class ClearanceCostInformation : public rtcus_navigation::trajectory_clearance::DefaultClearanceCost
{
public:
  ClearanceCostInformation() :
      clearance_(std::numeric_limits<t_float>::quiet_NaN()), trajectory_(NULL)
  {
  }
  double clearance_;
  const std::vector<rtcus_nav_msgs::DynamicState2D>* trajectory_;
  inline void setClearance(t_float value)
  {
    this->clearance_ = value;
  }
};

template<typename ActionType>
  class CommandCost : public ClearanceCostInformation
  {
  protected:
    t_float heading_;
    t_float velocity_;
    t_float total_;
    t_float nonadmisiblity_;

  public:

    CommandCost() :
        ClearanceCostInformation()
    {
    }
    virtual ~CommandCost()
    {
    }

    virtual void initialize(const ActionType& command)
    {
      this->nonadmisiblity_ = std::numeric_limits<t_float>::quiet_NaN();
      this->clearance_ = std::numeric_limits<t_float>::quiet_NaN();
      this->heading_ = std::numeric_limits<t_float>::quiet_NaN();
      this->total_ = std::numeric_limits<t_float>::quiet_NaN();
      this->velocity_ = std::numeric_limits<t_float>::quiet_NaN();
    }

    /*For lookup tables*/
    virtual double getNormalizedIndexationFactor() const=0;

    virtual void computeAdmisibility(
        const rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig& kinodynamic_constraints)=0;

    virtual const ActionType& getAction() const=0;

    inline t_float getAngularCollisionDistance() const
    {
      return angular_collision_distance_;
    }

    inline t_float getLinearCollisionDistance() const
    {
      return linear_collision_distance_;
    }

    inline void setNonAdmisibleCommand(double nonadmisiblity)
    {
      RTCUS_ASSERT(nonadmisiblity>=0 && nonadmisiblity<=1.0);
      this->nonadmisiblity_ = nonadmisiblity;
    }

    inline bool isAdmisibleCommand() const
    {
      return nonadmisiblity_ != 1.0;
    }

    inline t_float getNonAdmisibility() const
    {
      return nonadmisiblity_;
    }

    inline t_float getClearance() const
    {
      return clearance_;
    }

    inline void setHeading(t_float value)
    {
      heading_ = value;
    }

    inline t_float getHeading() const
    {
      return heading_;
    }

    inline void setVelocity(t_float value)
    {
      velocity_ = value;
    }

    inline t_float getVelocity() const
    {
      return velocity_;
    }

    inline void setTotalCost(t_float value)
    {
      this->total_ = value;
    }

    inline t_float getTotalCost() const
    {
      return total_;
    }

  };

class DwaCommandCost : public rtcus_nav_msgs::Twist2D, public CommandCost<rtcus_nav_msgs::Twist2D>
{

protected:
  t_float kurvature_alpha_;
public:
  DwaCommandCost() :
      CommandCost<rtcus_nav_msgs::Twist2D>()
  {

  }
  virtual void initialize(const rtcus_nav_msgs::Twist2D& cmd)
  {
    CommandCost<rtcus_nav_msgs::Twist2D>::initialize(cmd);
    this->linear = cmd.linear;
    this->angular = cmd.angular;
    this->kurvature_alpha_ = atan2(linear, angular);
  }

  inline t_float getV() const
  {
    return this->linear;
  }

  inline t_float getOmega() const
  {
    return this->angular;
  }

  virtual double getNormalizedIndexationFactor() const
  {
    return this->getKurvatureAlpha();
  }

  inline t_float getKurvatureAlpha() const
  {
    return kurvature_alpha_;
  }
  virtual const rtcus_nav_msgs::Twist2D& getAction() const
  {
    return *this;
  }

  inline void computeAngularCollisionDistance(t_float linear_collision_distance)
  {
    this->linear_collision_distance_ = linear_collision_distance;
    if (fabs(angular) < 0.0002 || linear_collision_distance == std::numeric_limits<t_float>::max())
      this->angular_collision_distance_ = std::numeric_limits<t_float>::max();
    else
    {
      t_float radio = fabs(linear / angular);
      this->angular_collision_distance_ = this->linear_collision_distance_ / radio;

    }
  }

  inline bool getCollisionPoint(PointXY& collision, double angular, double linear) const
  {
    if (this->linear_collision_distance_ == numeric_limits<t_float>::max())
      return false;
    else
    {
      if (fabs(angular) < 0.0002)
      {
        collision.y = 0;
        collision.x = this->linear_collision_distance_;
      }
      else
      {
        t_float radio = linear / angular;
        collision.x = fabs(radio) * sin(angular_collision_distance_);
        collision.y = radio - radio * cos(angular_collision_distance_);
      }
      return true;
    }
  }

  virtual void computeAdmisibility(
      const rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig& kinodynamic_constraints)
  {
    //---------------ADMISIBIILITY CHECKING ------------------------------------
    if (this->collision)
    {
      this->computeAngularCollisionDistance(this->linear_collision_distance_);
      // v<=sqrt(2*dist(v,omega)·velocity_brake_aceleration) and omega <=sqrt(2*dist(w,omega)*omega_brake_acceleration)
      // va is the set of velocieties (v,omega) which allow the robot to stop without colliding with an obstacle.

      //admisibility <- (result.getV() * result.getV()) < (2 * result.getLinearCollisionDistance() * config_.getKinodynamicConfig().linear_brake_limit);
      double linear_non_admisibility_ratio = std::min(
          1.0, (getV() * getV()) / (2 * getLinearCollisionDistance() * kinodynamic_constraints.linear_brake_limit));

      double angular_non_admisibility_ratio = std::min(
          1.0,
          (getOmega() * getOmega())
              / (2 * getAngularCollisionDistance() * fabs(kinodynamic_constraints.angular_acceleration_limit)));

      float non_admisibility_ratio = max(angular_non_admisibility_ratio, linear_non_admisibility_ratio);
      RTCUS_ASSERT(non_admisibility_ratio >= 0.0 && non_admisibility_ratio <= 1.0);
      RTCUS_ASSERT(non_admisibility_ratio >= 0.0 && non_admisibility_ratio <= 1.0);
      this->nonadmisiblity_ = non_admisibility_ratio;
    }
    else
      this->nonadmisiblity_ = 0.0;
  }

public:
  friend std::ostream& operator<<(std::ostream &out, rtcus_dwa::DwaCommandCost &command_cost);
};

}

#endif /* DWA_command__COST_H_ */
