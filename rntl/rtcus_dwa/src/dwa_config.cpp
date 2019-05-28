/*
 * dwa_config.cpp
 *
 *  Created on: Oct 7, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_dwa/dwa_config.h>
#include <algorithm>
#include <ros/ros.h>

namespace rtcus_dwa
{
using namespace std;

DwaConfig::DwaConfig()
{
  //these values allows reach all posbile commands
  //just testing
  this->kinodynamic_configuration_.linear_acceleration_limit = 0.0;
  this->kinodynamic_configuration_.linear_brake_limit = 0.0;
  this->kinodynamic_configuration_.linear_backwards_speed_limit = 0.0;
  this->kinodynamic_configuration_.linear_forward_speed_limit = 0.0;

  //max angular velocity +- PI
  this->kinodynamic_configuration_.angular_speed_limit = 0.0;

  this->kinodynamic_configuration_.angular_acceleration_limit = 0.0;

  v_step = (kinodynamic_configuration_.linear_forward_speed_limit
      - this->kinodynamic_configuration_.linear_backwards_speed_limit) / 20.0;

  max_collision_distance = 500;
  omega_step = 0.2;
  no_step = true;
  v_res = 100;
  omega_res = 100;

  simulation_time_step = 0.01;

  k_clearance = 1.0;
  k_heading = 10.0;
  k_velocity = 1.0;
  security_area_obstacle_inflation = 0.7;
  security_area_clearance_weight_obsolete = 0.25;
  clearance_sigmoid_location = 0.3;
  clearance_sigmoid_scale = 0.5;

  update_window(0, 0, simulation_time_step);

  this->has_collision_ = false;
}
;

rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig DwaConfig::setKinoDynamicParameters(
    const rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig& config_update)
{
  this->kinodynamic_configuration_ = config_update;

  if (this->kinodynamic_configuration_.linear_forward_speed_limit
      - this->kinodynamic_configuration_.linear_backwards_speed_limit < this->v_step)
    this->kinodynamic_configuration_.linear_forward_speed_limit = this->v_step;

  this->onUpdated(*this);
  return this->kinodynamic_configuration_;
}

const SimpleDwaConfig& DwaConfig::updateConfig(const SimpleDwaConfig& config_update)
{
  ((SimpleDwaConfig&)*this) = config_update;

  RTCUS_ASSERT(this->kinodynamic_configuration_.linear_backwards_speed_limit>=0);

  /*Minimum forwards velocity allowed*/
  if (this->kinodynamic_configuration_.linear_forward_speed_limit
      + this->kinodynamic_configuration_.linear_backwards_speed_limit < this->v_step)
    this->kinodynamic_configuration_.linear_forward_speed_limit = this->v_step;

  RTCUS_ASSERT_MSG(
      this->kinodynamic_configuration_.linear_backwards_speed_limit>=0,
      "The backward speed has to be specified in positive meters per seconds. [linear_backward_speed_limit: %lf]", this->kinodynamic_configuration_.linear_backwards_speed_limit);

  this->onUpdated(*this);
  return *this;
}

bool DwaConfig::update_window(t_float v_actual, t_float omega_actual, double control_time_period)
{

  //------------- compute velocity window linear velocity limits ---------------------
  if (v_actual < 0)
  {
    //TODO: Improve this
    ROS_WARN("Negative Speed. System does not implement backward speed, considering current speed v<0.");
    v_actual = 0;
  }

  if (v_actual < -this->kinodynamic_configuration_.linear_backwards_speed_limit
      || v_actual > this->kinodynamic_configuration_.linear_forward_speed_limit)
  {
    v_actual = min(max(v_actual, -this->kinodynamic_configuration_.linear_backwards_speed_limit),
                   (t_float)kinodynamic_configuration_.linear_forward_speed_limit);
    ROS_WARN_THROTTLE(
        1.0,
        "The current linear speed (%lf) is out of the supposed kinematic limits: [%lf,%lf]. The limit speed will be considered to built the dynamic window ", v_actual, this->kinodynamic_configuration_.linear_forward_speed_limit, - this->kinodynamic_configuration_.linear_backwards_speed_limit);
  }

  if (omega_actual < -this->kinodynamic_configuration_.angular_speed_limit
      || omega_actual > this->kinodynamic_configuration_.angular_speed_limit)
  {
    omega_actual = min(max(omega_actual, -this->kinodynamic_configuration_.angular_speed_limit),
                       this->kinodynamic_configuration_.angular_speed_limit);
    ROS_WARN_THROTTLE(
        1.0,
        "The current angular speed (%lf) is out of the supposed kinematic limits: [%lf,%lf]. The limit speed will be considered to built the dynamic window ", omega_actual, this->kinodynamic_configuration_.angular_speed_limit, - this->kinodynamic_configuration_.angular_speed_limit);
  }

  //GO FORWARD DYNAMIC WINDOW
  if (v_actual >= 0)
  {

    //decrease abs(speed)
    //stop or dynamic window bottom through braking
    v_bottom_ = min(
        max((t_float)(v_actual - this->kinodynamic_configuration_.linear_brake_limit * control_time_period),
            (t_float)0.0),
        (t_float)kinodynamic_configuration_.linear_forward_speed_limit);

    //increase abs(speed)
    //max velocity or dynamic window top
    v_top_ = min(
        max((t_float)(v_actual + this->kinodynamic_configuration_.linear_acceleration_limit * control_time_period),
            (t_float)0.0),
        (t_float)kinodynamic_configuration_.linear_forward_speed_limit);
  }
  //GO BACKWARD DYNAMIC WINDOW
  else
  {

    //decrease abs(speed)
    //stop or dynamic window top through braking
    v_top_ = min((t_float)(v_actual + this->kinodynamic_configuration_.linear_brake_limit * control_time_period),
                 (t_float)0.0);
    v_bottom_ = max(
        (t_float)(v_actual - this->kinodynamic_configuration_.linear_acceleration_limit * control_time_period),
        (t_float)this->kinodynamic_configuration_.linear_backwards_speed_limit);
  }

  //this is needed because a noise odometry reading stopped can built "an inversed window"
  v_top_ = std::max(v_top_, v_bottom_);

  //------------- compute velocity window linear velocity limits ---------------------
  omega_right_ = min(
      (t_float)(omega_actual + this->kinodynamic_configuration_.angular_acceleration_limit * control_time_period),
      (t_float)kinodynamic_configuration_.angular_speed_limit);
  omega_left_ = max(
      (t_float)(omega_actual - this->kinodynamic_configuration_.angular_acceleration_limit * control_time_period),
      (t_float)-kinodynamic_configuration_.angular_speed_limit);

  //this is needed because a noise odometry reading stopped can built "an inversed window"
  omega_right_ = std::max(omega_left_, omega_right_);

  //-----------------------------------------------------------------
  if (no_step)
  {
    v_step = (v_top_ - v_bottom_) / (t_float)v_res;
    omega_step = (omega_right_ - omega_left_) / (t_float)omega_res;
  }

  if (omega_step > 0 && omega_right_ > omega_left_)
    resolution_width = (omega_right_ - omega_left_) / omega_step;
  else
    resolution_width = 1;

  if (v_step > 0 && v_top_ > v_bottom_)
    resolution_height = (v_top_ - v_bottom_) / v_step;
  else
    resolution_height = 1;

  full_u_space_resolution_omega_ = 2.0 * kinodynamic_configuration_.angular_speed_limit / omega_step;
  full_u_space_resolution_velocity_ = kinodynamic_configuration_.linear_forward_speed_limit / v_step;

  RTCUS_ASSERT_MSG(
      (omega_left_ + omega_step * (resolution_width - 1.0) <= omega_right_) && (v_bottom_ + v_step * (resolution_height - 1.0) <= v_top_),
      "Incoherent dynamic window configuration: omega_left %lf, omega_step %lf, omega_right %lf ||| v_bottom %lf v_step %lf v_top %lf", omega_left_, omega_step, omega_right_, v_bottom_, v_step, v_top_);

  return true;
}

t_float DwaConfig::get_max_linear_velocity() const
{
  return max(fabs(this->v_top_), fabs(this->v_bottom_));
}

t_float DwaConfig::get_max_angular_velocity() const
{
  return max(fabs(this->omega_left_), fabs(this->omega_right_));
}

void DwaConfig::getCommandCoordinates(const t_float v, const t_float omega, int & pi, int& pj) const
{
  pi = -1;
  pj = -1;
  bool error = false;
  if (v_top_ == v_bottom_)
    pi = 0;
  else if (v < v_bottom_ || v > v_top_)
  {
    ROS_DEBUG(
        "DWA float rounding error while getting  i,j command from v, omega. Selected velocity %lf outside of the window %lf %lf", v, v_bottom_, v_top_);
    error = true;
  }

  if (omega_left_ == omega_right_)
    pj = 0;
  else if (omega < omega_left_ || omega > omega_right_)
  {
    ROS_DEBUG(
        "DWA_ERROR float rounding error while getting  i,j command from v, omega. Selected angular velocity %lf outside of the window %lf %lf", omega, omega_left_, omega_right_);
    error = true;
  }
  RTCUS_ASSERT(v >= v_bottom_ && v <= v_top_ && omega >= omega_left_ && omega <= omega_right_);

  if (!error)
  {
    pi = floor(((v - v_bottom_) / (v_top_ - v_bottom_)) * v_res);
    pj = floor((omega - omega_left_) / (omega_right_ - omega_left_) * omega_res);
  }

}
}
