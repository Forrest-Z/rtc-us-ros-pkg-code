/*
 * dynamic_window_action_space_visual_representation.h
 *
 *  Created on: Oct 25, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef DYNAMIC_WINDOW_ACTION_SPACE_VISUAL_REPRESENTATION_H_
#define DYNAMIC_WINDOW_ACTION_SPACE_VISUAL_REPRESENTATION_H_

#include <rtcus_navigation_tools/visual_representations/visual_representation.h>
#include <rtcus_dwa/common.h>
#include <rtcus_nav_msgs/Twist2D.h>
#include <cv_bridge/cv_bridge.h>

namespace rtcus_dwa
{
namespace visual_representation
{
using namespace rtcus_navigation_tools;

template<typename GoalType>
  class DynamicWindowActionSpaceRepresentation : public VisualRepresetationBase
  {
  private:
    cv::Mat original_byte;
  protected:
    //boost::shared_ptr<sensor_msgs::Image> image_msg_;
    ros::Publisher admisibility_map_pub_;
    ros::Publisher previous_action_space_cost_;
    ros::Publisher admisibility_danger_pub_;

    rtcus_nav_msgs::Twist2D best_command_;
    cv_bridge::CvImage attenuation_msg;
    cv_bridge::CvImage original_image_msg;
    cv_bridge::CvImage resulting_image_msg;
    cv_bridge::CvImage admisibility_danger_;
    cv_bridge::CvImage full_u_space;
    const cv::Scalar BLUE_HELPER_COLOR;

  public:
    virtual ~DynamicWindowActionSpaceRepresentation();

    DynamicWindowActionSpaceRepresentation(const std::string& representation_frame, ros::NodeHandle& nh);
    virtual void update(const rtcus_nav_msgs::Twist2D& best_command);

    virtual void represent(const rtcus_dwa::DwaLocalPlannerBase<GoalType> & target_object_);
  }
  ;
}
}

#endif /* DYNAMIC_WINDOW_ACTION_SPACE_VISUAL_REPRESENTATION_H_ */
