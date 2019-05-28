/*
 * Timer.h
 *
 *  Created on: Dec 17, 2011
 *      Author: geus
 */

#ifndef TIMER_H_
#define TIMER_H_

//http://stackoverflow.com/questions/3797708/millisecond-accurate-benchmarking-in-c

#include <ros/ros.h>
#include <list>
#include <map>
#include <rtcus_timing/TimingEvent.h>

using namespace ros;

namespace rtcus_timing
{

struct timing_entry;

class Timer
{
private:

  Publisher event_pub_;
  NodeHandle nh_;
  NodeHandle nh_private;
  std::map<std::string, std::list<timing_entry> > timing_data;
  std::map<std::string, Publisher> publishers;
  bool system_time;
  bool enable;

public:
  //node prefix by default ~
  Timer(std::string node_prefix = "~", bool using_system_time = false);
  //key default value = "default"
  void start(std::string key = "default");
  //key default value = "default"
  ros::Duration stop(std::string key = "default");
  //key default value = "default"
  ros::Duration last_duration(std::string key = "default");
  //publishes all the cumulated timmings on the "timing" topic
  void publish_data();
  void configuration_refresh();
};

struct timing_entry
{
  ros::Time start_time;
  ros::Time end_time;
  bool closed;

  ros::Duration getDuration()
  {
    return end_time - start_time;
  }
};

}

#endif /* TIMER_H_ */
