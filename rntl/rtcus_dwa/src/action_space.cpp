/*
 * action_space.cpp
 *
 *  Created on: Oct 10, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */
#include <rtcus_dwa/action_space.h>
#include <rtcus_assert/rtcus_assert.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <rtcus_nav_msgs/Twist2D.h>
#include <std_msgs/Float32.h>

namespace rtcus_dwa
{

using namespace rtcus_nav_msgs;

DwaActionSpace::DwaActionSpace() :
    use_method_nonadmisibility_repulsion_(true), use_method_clearance_cost_dilation_(true), use_method_clearance_streatch_(
        true), use_secure_speed_in_non_admisibility_(true), use_dynamic_clearance_importance_from_admisibilty_danger_(
        true), use_non_admisibility_degree_as_clearance_(false), use_distance_as_clerarance_(false), nh_(
        "~/planner/high_clearance_dwa"), admisibility_img_(std_msgs::Header(), "mono8")

{
  this->global_non_admisibility_degree_pub_ = nh_.advertise<std_msgs::Float32>("global_non_admisibility_degree", 1);

  this->reset();

}
void DwaActionSpace::reset()
{

  if (!nh_.getParam("use_method_non_admisibilty_repulsion", this->use_method_nonadmisibility_repulsion_))
    nh_.setParam("use) =_method_non_admisibilty_repulsion", this->use_method_nonadmisibility_repulsion_);

  if (!nh_.getParam("use_method_clearance_cost_dilation", this->use_method_clearance_cost_dilation_))
    nh_.setParam("use_method_clearance_cost_dilation", this->use_method_clearance_cost_dilation_);

  if (!nh_.getParam("use_non_admisibility_degree_as_clearance", this->use_non_admisibility_degree_as_clearance_))
    nh_.setParam("use_non_admisibility_degree_as_clearance", this->use_non_admisibility_degree_as_clearance_);

  if (!nh_.getParam("use_distance_as_clerarance", this->use_distance_as_clerarance_))
    nh_.setParam("use_distance_as_clerarance", this->use_distance_as_clerarance_);

  if (!nh_.getParam("use_secure_speed_in_non_admisibility", this->use_secure_speed_in_non_admisibility_))
    nh_.setParam("use_secure_speed_in_non_admisibility", this->use_secure_speed_in_non_admisibility_);

  if (!nh_.getParam("use_dynamic_clearance_importance_from_admisibility_danger",
                    this->use_dynamic_clearance_importance_from_admisibilty_danger_))
    nh_.setParam("use_dynamic_clearance_importance_from_admisibility_danger",
                 this->use_dynamic_clearance_importance_from_admisibilty_danger_);

  if (!nh_.getParam("use_method_heading_admisibility", this->use_method_heading_admisibility_))
    nh_.setParam("use_method_heading_admisibility", this->use_method_heading_admisibility_);

  admisibility_img_.image.create(100, 100, CV_8U);
  admisibility_attenuation_map_.create(100, 100, CV_32FC1);
  resulting_cost_map_.create(100, 100, CV_32FC3);
  original_cost_map_.create(100, 100, CV_32FC3);
  admisibility_danger_.create(100, 100, CV_32FC1);
}

DwaActionSpace::~DwaActionSpace()
{
}

void DwaActionSpace::initialize(const DwaConfig& config)
{
  rtcus_nav_msgs::Twist2D current_action;
  current_action.linear = config.get_v_botom();
  unsigned int action_index = 0;
  for (unsigned int i = 0; i < config.get_resolution_height(); current_action.linear += config.v_step, i++)
  {
    current_action.angular = config.get_omega_left();
    for (unsigned int j = 0; j < config.get_resolution_width();
        current_action.angular += config.omega_step, j++, action_index++)
    {
      RTCUS_ASSERT_MSG(
          current_action.linear<=config.getKinodynamicConfig().linear_forward_speed_limit && current_action.linear>=-config.getKinodynamicConfig().linear_backwards_speed_limit &&current_action.angular>= - config.getKinodynamicConfig().angular_speed_limit &&current_action.angular<= config.getKinodynamicConfig().angular_speed_limit,
          "Action Kinematically Incoherent, action %d of %d. v %lf omega %lf. v in [%lf %lf] omega in [%lf %lf]", action_index, size(), current_action.linear, current_action.angular, -config.getKinodynamicConfig().linear_backwards_speed_limit, config.getKinodynamicConfig().linear_forward_speed_limit, - config.getKinodynamicConfig().angular_speed_limit, config.getKinodynamicConfig().angular_speed_limit);
      CommandCost<Twist2D> &command_cost = this->getAction(i * config.get_resolution_width() + j);
      command_cost.initialize(current_action);
    }
  }
}

//--------------------------------------------------------------------

CommandCost<rtcus_nav_msgs::Twist2D>& DwaActionSpace::getAction(unsigned int i)
{
  return data_[i];
}

const CommandCost<rtcus_nav_msgs::Twist2D>& DwaActionSpace::getAction(unsigned int i) const
{
  return data_[i];
}

unsigned int DwaActionSpace::size() const
{
  return data_.size();
}

const CommandCost<rtcus_nav_msgs::Twist2D>& DwaActionSpace::back() const
{
  return data_.back();
}

const CommandCost<rtcus_nav_msgs::Twist2D>& DwaActionSpace::front() const
{
  return data_.front();
}

void DwaActionSpace::onConfigUpdate(const DwaConfig& config)
{
  this->config_ = &config;

  RTCUS_ASSERT_MSG(config.get_resolution_width() > 0 && config.get_resolution_height() > 0,
                   "The selected velocity subspace is empty.");
  unsigned int window_len = config.get_resolution_width() * config.get_resolution_height();

  if (data_.size() != window_len || (unsigned int)admisibility_img_.image.rows != config.get_resolution_height()
      || (unsigned int)admisibility_img_.image.cols != config.get_resolution_width())
  {
    ROS_INFO(
        "Updating action space map image to resolution rows %d cols %d", this->config_->get_resolution_height(), this->config_->get_resolution_width());

    data_.resize(window_len);
    admisibility_img_.image.create(this->config_->get_resolution_height(), this->config_->get_resolution_width(),
                                   CV_8U);

    admisibility_danger_.create(this->config_->get_resolution_height(), this->config_->get_resolution_width(),
                                CV_32FC1);

    admisibility_attenuation_map_.create(this->config_->get_resolution_height(), this->config_->get_resolution_width(),
                                         CV_32FC1);
    original_cost_map_.create(this->config_->get_resolution_height(), this->config_->get_resolution_width(), CV_32FC3);
    resulting_cost_map_.create(this->config_->get_resolution_height(), this->config_->get_resolution_width(), CV_32FC3);

  }
}

inline float DwaActionSpace::getClearance(const CommandCost<Twist2D>& command_cost)
{
  t_float selected_clearance;
  if (use_non_admisibility_degree_as_clearance_)
    selected_clearance = command_cost.getClearance() * (1 - command_cost.getNonAdmisibility())
        + command_cost.getNonAdmisibility();
  else if (use_distance_as_clerarance_)
    selected_clearance = std::max(
        0.0,
        (this->config_->max_collision_distance - command_cost.getLinearCollisionDistance())
            / this->config_->max_collision_distance);
  else
    selected_clearance = command_cost.getClearance();

  return selected_clearance;
}

bool DwaActionSpace::updateCostAndAttenuationMaps(double& clearance_scale, double& min_clearance)
{
  double max_clearance;
  max_clearance = 0.0, min_clearance = 1.0;

  admisibility_saturation_mean_ = 0;
  non_admisible_action_found_ = false;
  this->min_collision_distance_ = numeric_limits<t_float>::max();
  this->percent_danger_value_ = 0.0;
  this->percent_secure_value_ = 0.0;

  for (unsigned int i = 0, v = this->config_->get_v_top(); i < this->config_->get_resolution_height();
      v += this->config_->v_step, i++)
  {
    t_float omega = this->config_->get_omega_left();
    for (unsigned int j = 0; j < config_->get_resolution_width(); omega += this->config_->omega_step, j++)
    {
      //COPY TO NONADMISIBILITY IMAGE
      const DwaCommandCost &command_cost = data_[(i * config_->get_resolution_width() + j)];
      admisibility_saturation_mean_ += command_cost.getNonAdmisibility();
      this->admisibility_danger_.at<float>(i, j) = command_cost.getNonAdmisibility();

      //GET MINIMUM LINEAR DISTANCE
      if (command_cost.getLinearCollisionDistance() < this->min_collision_distance_)
        this->min_collision_distance_ = command_cost.getLinearCollisionDistance();

      //GET PERCENT NONADMSIBILITY DANGER OVER SPEED THRESHOLD
      if (command_cost.getNonAdmisibility() > this->config_->secure_velocity_saturation_admisibility_threshold)
        this->percent_danger_value_ += 1.0;

      if (command_cost.getNonAdmisibility() < this->config_->secure_velocity_saturation_admisibility_threshold)
        this->percent_secure_value_ += 1.0;

      if (command_cost.isAdmisibleCommand())
      {
        float selected_clearance = getClearance(command_cost);

        if (max_clearance < selected_clearance)
          max_clearance = selected_clearance;
        else if (min_clearance > selected_clearance)
          min_clearance = selected_clearance;

        //COPY TO COSTMAP IMAGE
        *(admisibility_img_.image.ptr < uchar > (i, j)) = 255;
        original_cost_map_.at<cv::Vec3f>(i, j) = cv::Vec3f(selected_clearance, command_cost.getHeading(),
                                                           command_cost.getVelocity());
      }
      else
      {
        this->non_admisible_action_found_ = true;
        *(admisibility_img_.image.ptr < uchar > (i, j)) = 0;
        original_cost_map_.at<cv::Vec3f>(i, j) = cv::Vec3f(1.0, 1.0, 1.0);
      }
    }
  }

  clearance_scale = (max_clearance - min_clearance);
  float total_elements = ((float)(config_->get_resolution_width() * config_->get_resolution_height()));
  this->percent_danger_value_ = std::max(0.0f, std::min(1.0f, this->percent_danger_value_ / total_elements));
  this->percent_secure_value_ = std::max(0.0f, std::min(1.0f, this->percent_secure_value_ / total_elements));
  this->admisibility_saturation_mean_ = admisibility_saturation_mean_ / total_elements;
  return admisibility_saturation_mean_ == 1;
}

void DwaActionSpace::compute_clearance_dilation_cost()
{
  /*METHOD FIND LOCAL MAXIMUM*/
  //cv::Mat window_kernel(cv::Size(1, 1), CV_8U);
  //cv::dilate(original_clearance_channel_, maximums_cost_map_, window_kernel, cv::Point(-1, -1), 1, cv::BORDER_REFLECT);
  //cv::Mat temp(maximums_cost_map_.rows, maximums_cost_map_.cols, CV_8U);
  //cvAnd(&maximums_cost_map_, &original_clearance_channel_, &temp);
  /*--- METHOD A: ACTION SPACE DILATION -----------------*/
  if (use_method_clearance_cost_dilation_)
  {
    RTCUS_ASSERT(config_->security_area_obstacle_inflation >= 0 && config_->security_area_obstacle_inflation <= 1.0);
    unsigned int v_level = config_->security_area_obstacle_inflation * config_->v_res;
    if (v_level % 2 == 0)
      v_level++;
    unsigned int omega_level = config_->security_area_obstacle_inflation * config_->omega_res;
    if (omega_level % 2 == 0)
      omega_level++;
    if (v_level >= 1 && omega_level >= 1)
    {
      cv::Mat window_kernel(cv::Size(omega_level, v_level), CV_8U);
      cv::dilate(original_cost_map_, resulting_cost_map_, window_kernel, cv::Point(-1, -1), 1, cv::BORDER_REFLECT);
    }
  }
}

void DwaActionSpace::compute_non_admisibility_repulsion()
{
  /*--- METHOD B: ADDING NON ADMISIBILITY REPULSION IN THE V-SPACE ----- */
  if (use_method_nonadmisibility_repulsion_)
  {
    cv::distanceTransform(admisibility_img_.image, admisibility_attenuation_map_, CV_DIST_L2, 3);
    double rep_admis_min = std::numeric_limits<double>::max();
    double rep_admis_max = std::numeric_limits<double>::min();
    //cv::minMaxIdx(this->admisibility_attenuation_map_, &rep_admis_min, &rep_admis_max);
    //ROS_INFO("[PRE] Admissibility repulsion map minimax: %f %f", (float)rep_admis_min, (float)rep_admis_max);
    cv::normalize(this->admisibility_attenuation_map_, this->admisibility_attenuation_map_, 0.0, 1.0, cv::NORM_MINMAX);
    cv::minMaxIdx(this->admisibility_attenuation_map_, &rep_admis_min, &rep_admis_max);
    //ROS_INFO("[AFTER] Admissibility repulsion map minimax: %f %f", (float)rep_admis_min, (float)rep_admis_max);
  }

}

void DwaActionSpace::compute_getWeightFactors(t_float& clearance_factor, t_float& heading_factor,
                                              t_float& velocity_factor)
{
  float clearance_gain = 0.0;
  if (this->use_dynamic_clearance_importance_from_admisibilty_danger_)
  {
    clearance_gain = this->admisibility_saturation_mean_;
    //TRICK... TODO IMPROVE:
    float too_close = this->min_collision_distance_ / (0.1 + this->config_->k_clearance_danger);
    if (too_close <= 1.0)
    {
      clearance_gain = max(this->admisibility_saturation_mean_,
                           this->admisibility_saturation_mean_ * (too_close) + (1 - too_close));
    }
  }

  //t_float total_factor_normalization = this->config_->k_clearance + (1.0 - clearance_gain) * this->config_->k_velocity
  //    + (1.0 - clearance_gain) * this->config_->k_heading;

  t_float total_factor_normalization = this->config_->k_clearance + this->config_->k_velocity
      + (1.0 - clearance_gain) * this->config_->k_heading;

  clearance_factor = this->config_->k_clearance / total_factor_normalization;
  heading_factor = (1.0 - clearance_gain) * this->config_->k_heading / total_factor_normalization;
  velocity_factor = this->config_->k_velocity / total_factor_normalization;

  RTCUS_ASSERT_MSG(
      clearance_factor+heading_factor+velocity_factor < 1.1,
      "normalization_factor: %lf, k_c: %lf, k_h: %lf, k_v: %lf", (double)total_factor_normalization, (double)clearance_factor, (double)heading_factor, (double)velocity_factor);
  //velocity_factor = (1.0 - clearance_gain) * this->config_->k_velocity / total_factor_normalization;
}

void DwaActionSpace::globalEvaluate()

{
  double clearance_scale, min_clearance_value;
  updateCostAndAttenuationMaps(clearance_scale, min_clearance_value);
  this->compute_clearance_dilation_cost();
  this->compute_non_admisibility_repulsion();
  t_float clearance_factor, heading_factor, velocity_factor;
  this->compute_getWeightFactors(clearance_factor, heading_factor, velocity_factor);

  /*-------- RECONSTRUCTION OF COSTS ----------------------------*/
  t_float v = this->config_->get_v_botom();
  for (unsigned int i = 0; i < this->config_->get_resolution_height(); v += this->config_->v_step, i++)
  {
    t_float omega = this->config_->get_omega_left();
    for (unsigned int j = 0; j < config_->get_resolution_width(); omega += this->config_->omega_step, j++)
    {
      unsigned int action_index = i * config_->get_resolution_width() + j;
      CommandCost<Twist2D> &command_cost = this->getAction(action_index);

      float new_clearance = this->getClearance(command_cost);
      if (!command_cost.isAdmisibleCommand())
      {
        command_cost.setTotalCost(1.0);
        command_cost.setClearance(1.0);
      }
      else
      {

        //METHOD A: ACTION SPACE DILATION
        if (use_method_clearance_cost_dilation_)
        {
          const cv::Vec3f& action_cost_pixel = resulting_cost_map_.at<cv::Vec3f>(i, j);
          new_clearance = action_cost_pixel[0];
        }
        else
        {
          //redundant?

        }

        //METHOD C: STRETCH-EQUALIZATION OF THE CLEARANCE COST
        if (use_method_clearance_streatch_)
        {
          if (clearance_scale > 0)
          {
            new_clearance = ((new_clearance - min_clearance_value) / clearance_scale);
            new_clearance = std::min(1.0f, std::max(new_clearance, 0.0f));
          }
        }
        else
        {
          //IF NOT AT LEAST BOUND THE RESULTS
          new_clearance = std::max(0.0f, std::min(new_clearance, 1.0f));
        }

        //--------- CHECKING CLEARANCE COHERENCE ------------------------------

        RTCUS_ASSERT_MSG(
            new_clearance >= 0.0 && new_clearance <= 1.0,
            "The implementation of the system should be reviewed. This values are not expected. Scale factors| clearance: %lf , heading: %lf, velocity: %lf| [new clearance %lf]", clearance_factor, heading_factor, velocity_factor, new_clearance);
      }

      //--------------SPEED COST STRATEGY: at saturation, slow commands are not expensive---------

      //NONADMISIBILITY DANGER
      //float D = this->admisibility_saturation_mean_ * config_->secure_velocity_saturation_admisibility_threshold;
      float D = std::min(
          1.0,
          std::max(
              0.0,
              (this->admisibility_saturation_mean_ - config_->secure_velocity_saturation_admisibility_threshold)
                  / (1.0 - this->admisibility_saturation_mean_)));
      //float D = this->percent_danger_value_;
      //float D = 1.0 - this->percent_secure_value_;

      float new_velocity = command_cost.getVelocity();
      if (use_secure_speed_in_non_admisibility_)
      {
        /*
         //EMERGENCY BRAKE!!!
         if (admisibility_saturation_mean_ > config_->secure_velocity_saturation_admisibility_threshold)
         {
         //WARNING: The individual admisibility cannot be used. If not, non-admisible and slow velocities can be more recomendable than admisible ones
         *
         if (command_cost.getAction().linear >= 0)
         //THE Higher velocity the higher cost
         new_velocity = (command_cost.getAction().linear / config_->get_v_top())
         * (1.0 - command_cost.getNonAdmisibility()) + command_cost.getNonAdmisibility();
         else
         //THE Higher velocity the higher cost
         new_velocity = (command_cost.getAction().linear / config_->get_v_botom())
         * (1.0 - command_cost.getNonAdmisibility()) + command_cost.getNonAdmisibility();

         }
         else
         //new_velocity = command_cost.getVelocity();
         new_velocity = command_cost.getVelocity() * (1 - command_cost.getNonAdmisibility())
         + command_cost.getNonAdmisibility();


         */
        double brake_cost = ((command_cost.getAction().linear - config_->get_v_botom())
            / (config_->get_v_top() - config_->get_v_botom())) * D;
        double original_cost = command_cost.getVelocity() * (1.0 - D);
        new_velocity = brake_cost + original_cost;
        RTCUS_ASSERT(new_velocity >= 0 && new_velocity <= 1.0);
      }

      //------------- HEADING STRATEGY --------------------
      //Dilation does not affect to heading and velocity
      //change this if necesary to action_cost_pixel[1] and [2];
      float new_heading;

      if (use_method_heading_admisibility_)
      {
        new_heading = command_cost.getHeading() * (1 - command_cost.getNonAdmisibility())
            + command_cost.getNonAdmisibility();
      }
      else
        new_heading = command_cost.getHeading();

      //---- CLEARANCE STRATEGY
      //METHOD B: ADDING NON ADMISIBILITY REPULSION IN THE V-SPACE
      if (use_method_nonadmisibility_repulsion_)
      {
        if (non_admisible_action_found_) //if not, the repulsion map is not ok.
        {
          //float value = (((*admisibility_attenuation_map_.ptr<float>(i, j))));
          //non_admisibility_repulsion = 1.0 - value / admisibilityDistanceMaxVal;
          float non_admisibility_repulsion = (((*admisibility_attenuation_map_.ptr<float>(i, j))));
          RTCUS_ASSERT_MSG(
              non_admisibility_repulsion >= -0.9 && non_admisibility_repulsion <= 1.1,
              "Incorrect normalization of the attenuation repulsion Map: [For action %f %f]-> %f", command_cost.getAction().linear, command_cost.getAction().angular, non_admisibility_repulsion);
          non_admisibility_repulsion = std::max(0.0f, std::min(1.0f, non_admisibility_repulsion));

          float resuplsion_cost_influence = min(
              1.0, admisibility_saturation_mean_ * config_->non_admisibility_repulsion_weight);

          new_clearance = (1.0 - non_admisibility_repulsion) * resuplsion_cost_influence
              + new_clearance * (1.0 - resuplsion_cost_influence);
          //this is now implicit in the regulation of the non_admisibility factor
          new_velocity = (1.0 - non_admisibility_repulsion) * resuplsion_cost_influence
              + new_velocity * (1.0 - resuplsion_cost_influence);
          //this is now implicit in the regulation of the non_admisibility factor
          new_heading = (1.0 - non_admisibility_repulsion) * resuplsion_cost_influence
              + new_heading * (1.0 - resuplsion_cost_influence);
        }
      }

      //--------------- RECOMPTUTE TOTAL COST----------------------------
      command_cost.setClearance(min(1.0f, new_clearance));
      command_cost.setVelocity(min(1.0f, new_velocity));
      command_cost.setHeading(min(1.0f, new_heading));

      command_cost.setTotalCost(
          command_cost.getClearance() * clearance_factor + command_cost.getHeading() * heading_factor
              + command_cost.getVelocity() * velocity_factor);

      if (command_cost.getTotalCost() < 0.0 || command_cost.getTotalCost() > 1.0)
      {
        command_cost.setTotalCost(std::max(0.0, std::min(1.0, command_cost.getTotalCost())));
        ROS_ERROR(
            "The implementation of the system should be reviewed. This values are not expected. scale factors| clearance: %lf, heading: %lf, velocity: %lf", clearance_factor, heading_factor, velocity_factor);
      }

      /*------------- COHERENCE CHEK ---------------------*/
      RTCUS_ASSERT_MSG(command_cost.getClearance() >= 0.0 && command_cost.getClearance() <= 1.0,
                       "clearance after action space reduction out of range: %lf", command_cost.getClearance());
      RTCUS_ASSERT_MSG(command_cost.getHeading() >= 0.0 && command_cost.getHeading() <= 1.0,
                       "heading after action space reduction out of range: %lf", command_cost.getHeading());
      RTCUS_ASSERT_MSG(command_cost.getVelocity() >= 0.0 && command_cost.getVelocity() <= 1.0,
                       "velocity after action space reduction out of range: %lf", command_cost.getVelocity());

      /*RTCUS_ASSERT_MSG(
       command_cost.getTotalCost() >= 0.0 && command_cost.getTotalCost() <= 1.0,
       "The implementation of the system should be reviewed. This values are not expected. scale factors| clearance: %lf, heading: %lf, velocity: %lf", clearance_factor, heading_factor, velocity_factor);*/
    }
  }

  updateFinalCostMap();
}

inline float strategy_stretching(float original_value, float min_value, float clearance_scale)
{
  float new_clearance = ((original_value - min_value) / clearance_scale);
  new_clearance = std::min(1.0f, std::max(new_clearance, 0.0f));
  return new_clearance;
}

//-------------------------------------------------------------------------------
const cv::Mat& DwaActionSpace::getAdmisibilityDanger() const
{
  return this->admisibility_danger_;
}

const cv_bridge::CvImage& DwaActionSpace::getAdmisibilityMap() const
{
  return this->admisibility_img_;
}

const cv::Mat& DwaActionSpace::getAdmisibilityAttenuationFactorMap() const
{
  return this->admisibility_attenuation_map_;
}

const cv::Mat& DwaActionSpace::getOriginalActionCostMap() const
{
  return this->original_cost_map_;
}

const cv::Mat& DwaActionSpace::getFinalActionCostMap() const
{

  return this->resulting_cost_map_;
}

/**
 * Update the final image of the action space. Introspection proposes.
 * */
void DwaActionSpace::updateFinalCostMap()
{
  t_float clearance_factor, heading_factor, velocity_factor;
  this->compute_getWeightFactors(clearance_factor, heading_factor, velocity_factor);

  t_float v = this->config_->get_v_botom();
  for (unsigned int i = 0; i < this->config_->get_resolution_height(); v += this->config_->v_step, i++)
  {
    t_float omega = this->config_->get_omega_left();
    for (unsigned int j = 0; j < config_->get_resolution_width(); omega += this->config_->omega_step, j++)
    {
      CommandCost<Twist2D> &command_cost = data_[(i * config_->get_resolution_width() + j)];
      if (command_cost.isAdmisibleCommand())
      {
        resulting_cost_map_.at<cv::Vec3f>(i, j) = cv::Vec3f(clearance_factor * command_cost.getClearance(),
                                                            heading_factor * command_cost.getHeading(),
                                                            velocity_factor * command_cost.getVelocity());
      }
      else
        resulting_cost_map_.at<cv::Vec3f>(i, j) = cv::Vec3f(1.0, 1.0, 1.0);
    }
  }
  std_msgs::Float32 msg;
  msg.data = this->admisibility_saturation_mean_;
  this->global_non_admisibility_degree_pub_.publish(msg);
}
}

