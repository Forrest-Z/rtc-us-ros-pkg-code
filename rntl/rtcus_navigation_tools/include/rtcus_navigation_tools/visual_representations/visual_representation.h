/*
 * visual_representation_traits.h
 *
 *  Created on: Oct 24, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef VISUAL_REPRESENTATION_TRAITS_H_
#define VISUAL_REPRESENTATION_TRAITS_H_

#include <list>
#include <string>
#include <ros/ros.h>

namespace rtcus_navigation_tools
{

class VisualRepresetationBase
{
protected:
  ros::NodeHandle nh;
  ros::Publisher representation_publisher_;
  std::string representation_frame_name_;

  VisualRepresetationBase(const std::string& representation_frame_name, ros::NodeHandle& n) :
      nh(n), representation_frame_name_(representation_frame_name)
  {

  }

  VisualRepresetationBase(const std::string& representation_frame_name, ros::NodeHandle& n, ros::Publisher pub) :
      nh(n), representation_publisher_(pub), representation_frame_name_(representation_frame_name)
  {

  }

};

template<class T>
  class VisualRepresetation : public VisualRepresetationBase
  {

  public:
    void represent(const T& target_object)
    {

    }
  };
}

#endif /* VISUAL_REPRESENTATION_TRAITS_H_ */
