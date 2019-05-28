/*
 * action_space.h
 *
 *  Created on: Oct 10, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef ACTION_SPACE_H_
#define ACTION_SPACE_H_

#include <rtcus_dwa/dwa_command_cost.h>
#include <vector>
#include <rtcus_dwa/dwa_config.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>

namespace rtcus_dwa
{

/**
 * \brief defines the set of applicable actions given the configuration context (do we need the state?)
 * \remarks the action have to store themselves its cost. One good method is to extend the a base RosMsg ActionType to make
 * the implementation more compatible with the rest of generic component.
 * */
template<typename ActionType, typename ConfigType>
  class ActionSpace
  {
  public:
    virtual ~ActionSpace()
    {
    }
    virtual void initialize(const ConfigType& config)=0;
    virtual CommandCost<ActionType>& getAction(unsigned int i)=0;
    virtual const CommandCost<ActionType>& getAction(unsigned int i) const=0;
    virtual unsigned int size() const=0;
    virtual const CommandCost<ActionType>& back() const=0;
    virtual const CommandCost<ActionType>& front() const=0;
    virtual void reset()=0;
    virtual void onConfigUpdate(const ConfigType& config)=0;
    virtual void globalEvaluate()=0;

    /*\brief If not any of the possible actions are admissible the system is admissibility-saturated*/
    bool virtual admisibilitySaturated() const=0;

  };

class DwaActionSpace : public ActionSpace<rtcus_nav_msgs::Twist2D, DwaConfig>
{
private:
  bool use_method_nonadmisibility_repulsion_;
  bool use_method_clearance_cost_dilation_;
  bool use_method_clearance_streatch_;
  bool use_secure_speed_in_non_admisibility_;
  bool use_dynamic_clearance_importance_from_admisibilty_danger_;
  bool use_non_admisibility_degree_as_clearance_;
  bool use_method_heading_admisibility_;
  bool use_distance_as_clerarance_;
  ros::NodeHandle nh_;

  cv_bridge::CvImage admisibility_img_;
  cv::Mat admisibility_attenuation_map_;
  cv::Mat original_cost_map_;
  cv::Mat resulting_cost_map_;
  cv::Mat admisibility_danger_;

  const DwaConfig* config_;
  bool non_admisible_action_found_;
  float admisibility_saturation_mean_;
  float admisibility_saturation_max_;
  float min_collision_distance_;
  float percent_danger_value_;
  float percent_secure_value_;
  ros::Publisher global_non_admisibility_degree_pub_;

  inline float getClearance(const CommandCost<rtcus_nav_msgs::Twist2D>& command_cost);

  std::vector<DwaCommandCost> data_;

public:
  virtual CommandCost<rtcus_nav_msgs::Twist2D>& getAction(unsigned int i);
  virtual const CommandCost<rtcus_nav_msgs::Twist2D>& getAction(unsigned int i) const;
  virtual unsigned int size() const;
  virtual const CommandCost<rtcus_nav_msgs::Twist2D>& back() const;
  virtual const CommandCost<rtcus_nav_msgs::Twist2D>& front() const;

protected:

  bool updateCostAndAttenuationMaps(double& clearance_scale, double& min_clearance_value);
  void updateFinalCostMap();

  void compute_getWeightFactors(t_float& clearance_factor, t_float& heading_factor, t_float& velocity_factor);
  void compute_clearance_dilation_cost();
  void compute_clearance_values_stretching(double& clearance_scale, double& min_clearance_value);
  void compute_non_admisibility_repulsion();

public:

  DwaActionSpace();
  virtual void reset();
  virtual ~DwaActionSpace();
  virtual void initialize(const DwaConfig& config);
  virtual bool admisibilitySaturated() const
  {
    return admisibility_saturation_mean_ == 1;
  }
  inline float admisibility_saturation() const
  {
    return admisibility_saturation_mean_;
  }
  virtual void onConfigUpdate(const DwaConfig& config);
  virtual void globalEvaluate();

  //------------------- FOR REPRESENTATION -------------------------------
  const cv::Mat& getOriginalActionCostMap() const;
  const cv::Mat& getFinalActionCostMap() const;
  const cv_bridge::CvImage& getAdmisibilityMap() const;
  const cv::Mat& getAdmisibilityAttenuationFactorMap() const;
  const cv::Mat& getAdmisibilityDanger() const;
};
}

#endif /* ACTION_SPACE_H_ */
