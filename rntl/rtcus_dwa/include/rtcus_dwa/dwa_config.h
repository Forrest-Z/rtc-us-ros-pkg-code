/*
 * simple_dwa.h
 *
 *  Created on: 29/11/2011
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef DWA_CONFIG_H_
#define DWA_CONFIG_H_
#include <rtcus_dwa/SimpleDwaConfig.h>
#include <boost/signal.hpp>
#include <rtcus_dwa/common.h>
#include <rtcus_kinodynamic_description/NonHolonomicKinoDynamicsConfig.h>
#include <rtcus_robot_shapes/polygonal_robot.h>

namespace rtcus_dwa
{

using namespace rtcus_kinodynamic_description;
using namespace rtcus_robot_shapes;

class DwaConfig : public SimpleDwaConfig
{
private:
  bool has_collision_;
  t_float v_bottom_, v_top_, omega_right_, omega_left_;
  unsigned int full_u_space_resolution_omega_, full_u_space_resolution_velocity_;

  rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig kinodynamic_configuration_;
  rtcus_robot_shapes::PolygonalRobot robot_shape_;

public:
  DwaConfig();

  boost::signal<void(const DwaConfig& config)> onUpdated;

  bool update_window(t_float v_actual, t_float omega_actual, double control_time_period);

  NonHolonomicKinoDynamicsConfig setKinoDynamicParameters(const NonHolonomicKinoDynamicsConfig& config_update);
  const PolygonalRobot& setRobotShape(const PolygonalRobot& shape)
  {
    this->robot_shape_ = shape;
    return this->robot_shape_;

  }

  inline const PolygonalRobot& getRobotShape() const
  {
    return this->robot_shape_;
  }

  inline const NonHolonomicKinoDynamicsConfig& getKinodynamicConfig() const
  {
    return this->kinodynamic_configuration_;
  }

  const SimpleDwaConfig& updateConfig(const SimpleDwaConfig& config_update);
  unsigned int resolution_width, resolution_height;

  inline t_float get_v_botom() const
  {
    return v_bottom_;
  }

  inline t_float get_v_top() const
  {
    return v_top_;
  }

  inline t_float get_omega_left() const
  {
    return omega_left_;
  }

  inline t_float get_omega_right() const
  {
    return omega_right_;
  }

  inline unsigned int get_resolution_width() const
  {
    return resolution_width;
  }

  inline unsigned int get_resolution_height() const
  {
    return resolution_height;
  }

  inline unsigned int get_full_action_space_res_v() const
  {
    return full_u_space_resolution_velocity_;
  }
  inline unsigned int get_full_action_space_res_omega() const
  {
    return full_u_space_resolution_omega_;
  }

  t_float get_max_linear_velocity() const;

  t_float get_max_angular_velocity() const;

  void getCommandCoordinates(const t_float v, const t_float omega, int & pi, int& pj) const;

  inline void setCollisionState(bool hasCollision)
  {
    this->has_collision_ = hasCollision;
  }
  inline bool hasCollision() const
  {
    return has_collision_;
  }
};

}
#endif
