/*
 * dynamic_window_action_space_visual_represenation.cpp
 *
 *  Created on: Nov 2, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_dwa/visual_representation/dynamic_window_action_space_visual_representation.h>
#include <rtcus_nav_msgs/Pose2D.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <rtcus_dwa/dwa_config.h>
#include <rtcus_dwa/simple_dwa_ros.h>

namespace rtcus_dwa
{
namespace visual_representation
{
using namespace rtcus_navigation_tools;

template<typename GoalType>
  DynamicWindowActionSpaceRepresentation<GoalType>::~DynamicWindowActionSpaceRepresentation()
  {
  }

template<typename GoalType>
  DynamicWindowActionSpaceRepresentation<GoalType>::DynamicWindowActionSpaceRepresentation(
      const std::string& representation_frame, ros::NodeHandle& nh) :
      VisualRepresetationBase(representation_frame, nh, nh.advertise<sensor_msgs::Image>("action_space_map/cost", 1)), attenuation_msg(
          std_msgs::Header(), "mono8"), original_image_msg(std_msgs::Header(), "rgb8"), resulting_image_msg(
          std_msgs::Header(), "rgb8"), admisibility_danger_(std_msgs::Header(), "mono8"), full_u_space(
          std_msgs::Header(), "rgb8"), BLUE_HELPER_COLOR(64, 64, 255)
  {
    //this->image_msg_ = boost::make_shared<sensor_msgs::Image>();
    admisibility_map_pub_ = nh.advertise<sensor_msgs::Image>("action_space_map/admisibility_map", 1);
    previous_action_space_cost_ = nh.advertise<sensor_msgs::Image>("action_space_map/previous_cost", 1);
    admisibility_danger_pub_ = nh.advertise<sensor_msgs::Image>("action_space_map/admisibility_danger", 1);

  }

template<typename GoalType>
  void DynamicWindowActionSpaceRepresentation<GoalType>::update(const rtcus_nav_msgs::Twist2D& best_command)
  {
    best_command_ = best_command;
  }

void paintGoal(cv::Mat& image, unsigned int best_cmd_i, unsigned int best_cmd_j, cv::Scalar color)
{
  cv::Point3_<unsigned char> best_command = image.at<cv::Point3_<unsigned char> >(best_cmd_i, best_cmd_j);
  cv::circle(image, cv::Point(best_cmd_j, best_cmd_i), 2, color);
  cv::line(image, cv::Point(best_cmd_j, 0), cv::Point(best_cmd_j, image.rows), color);
  cv::line(image, cv::Point(0, best_cmd_i), cv::Point(image.cols, best_cmd_i), color);
  image.at<cv::Point3_<unsigned char> >(best_cmd_i, best_cmd_j) = best_command;

}

template<typename GoalType>
  void DynamicWindowActionSpaceRepresentation<GoalType>::represent(
      const rtcus_dwa::DwaLocalPlannerBase<GoalType> & target_object_)
  {
    ROS_DEBUG(" Publishing Image");
    const DwaConfig& config = target_object_.getConfig();
    int best_cmd_i, best_cmd_j;
    target_object_.getConfig().getCommandCoordinates(best_command_.linear,
                                                                               best_command_.angular, best_cmd_i,
                                                                               best_cmd_j);
    //----------------------------------------------------------------------------------------------

    DwaActionSpace& action_space = dynamic_cast<DwaActionSpace&>(*target_object_.action_space_);
    if ( this->representation_publisher_.getNumSubscribers() > 0)
    {
      const cv::Mat& final_image = action_space.getFinalActionCostMap();
      final_image.convertTo(this->resulting_image_msg.image, CV_8UC3, 255);
      paintGoal(resulting_image_msg.image, best_cmd_i, best_cmd_j, BLUE_HELPER_COLOR);
    }
    else
      target_object_.getConfig().getCommandCoordinates(best_command_.linear, best_command_.angular, best_cmd_i,
                                                       best_cmd_j);

    //----------------------------------------------------------------------------------------------

    {
      const cv::Mat& original_cost_img = action_space.getOriginalActionCostMap();
      original_cost_img.convertTo(original_byte, CV_8UC3, 255);
      cv::pyrUp(original_byte, this->original_image_msg.image);
      paintGoal(original_image_msg.image, 2 * best_cmd_i, 2 * best_cmd_j, BLUE_HELPER_COLOR);
      cv::flip(this->original_image_msg.image, this->original_image_msg.image, 0);
    }
    //-----------------------------------------------------------------------------------------
    if (this->admisibility_danger_pub_.getNumSubscribers() > 0)
    {
      const cv::Mat& admisibility_danger = action_space.getAdmisibilityDanger();
      admisibility_danger.convertTo(this->admisibility_danger_.image, CV_8U, 255);
    }

    //-------------------------- FULL ACTION-SPACE IMAGE ---------------------------------------------------------------
    if (this->previous_action_space_cost_.getNumSubscribers() > 0)
    {
      full_u_space.image = cv::Mat::zeros(target_object_.getConfig().get_full_action_space_res_v(),
                                          target_object_.getConfig().get_full_action_space_res_omega(), CV_8UC3);
      cv::line(full_u_space.image, cv::Point(full_u_space.image.cols / 2, 0),
               cv::Point(full_u_space.image.cols / 2, full_u_space.image.rows - 1), BLUE_HELPER_COLOR, 2);
      cv::line(full_u_space.image, cv::Point(0, full_u_space.image.rows / 2),
               cv::Point(full_u_space.image.cols - 1, full_u_space.image.rows / 2), BLUE_HELPER_COLOR, 2);

      cv::Point end = cv::Point(
          (config.getKinodynamicConfig().angular_speed_limit - config.get_omega_left()) / config.omega_step,
          config.get_v_top() / config.v_step);

      cv::Point origin = cv::Point(
          (config.getKinodynamicConfig().angular_speed_limit - config.get_omega_right()) / config.omega_step,
          config.get_v_botom() / config.v_step);

      //ROS_INFO(
      //    "origin %d %d -> end %d %d -> vstep %lf omegastep %lf", origin.x, origin.y, end.x, end.y, config.v_step, config.omega_step);

      cv::Mat dst_roi = full_u_space.image(cv::Rect(origin, end));
      //ROS_INFO(
      //    "dst roi rows %d cols %d while final image rows %d, cols %d", dst_roi.rows, dst_roi.cols, final_image.rows, final_image.cols);

      this->resulting_image_msg.image.copyTo(dst_roi);
      origin.x -= 1;
      origin.y -= 1;
      cv::rectangle(full_u_space.image, origin, end, BLUE_HELPER_COLOR);
    }
    //ROS_INFO( "image res (%d, %d)", full_u_space.image.rows, full_u_space.image.cols);
    //previous_action_space_cost_.publish(this->original_image_msg.toImageMsg());

    //---------------------------------------------------------------------------------------------

    if (this->admisibility_map_pub_.getNumSubscribers() > 0)
    {
      const cv::Mat& attenuation_img = action_space.getAdmisibilityAttenuationFactorMap();
      attenuation_img.convertTo(attenuation_msg.image, CV_8UC3, 255);
      admisibility_map_pub_.publish(attenuation_msg.toImageMsg());
    }

    //------------------------------------------------------------------
    //Optimize avoiding these flips if no subscribers
    if (this->previous_action_space_cost_.getNumSubscribers() > 0)
    {
      cv::flip(this->resulting_image_msg.image, this->resulting_image_msg.image, -1);
      representation_publisher_.publish(resulting_image_msg.toImageMsg());
    }

    if (this->previous_action_space_cost_.getNumSubscribers() > 0)
    {
      cv::flip(full_u_space.image, full_u_space.image, -1);
      previous_action_space_cost_.publish(full_u_space.toImageMsg());
    }
    if (this->admisibility_danger_pub_.getNumSubscribers() > 0)
    {
      cv::flip(this->admisibility_danger_.image, this->admisibility_danger_.image, -1);
      admisibility_danger_pub_.publish(this->admisibility_danger_.toImageMsg());
    }
  }

template class DynamicWindowActionSpaceRepresentation<rtcus_dwa::PointXY> ;
template class DynamicWindowActionSpaceRepresentation<rtcus_nav_msgs::Twist2D> ;

}
}
