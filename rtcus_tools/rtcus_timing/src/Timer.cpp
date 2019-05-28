#include "rtcus_timing/timer.h"
#include <sys/time.h>

namespace rtcus_timing
{

Timer::Timer(std::string node_prefix, bool using_system_time) :
    nh_(node_prefix), nh_private(node_prefix + "timing/")
{
  event_pub_ = nh_.advertise < rtcus_timing::TimingEvent > ("timing_agg", 100);
  system_time = using_system_time;
  enable = true;
  configuration_refresh();
}

void Timer::start(std::string key)
{
  //ROS_INFO("timer start -> %s",key.c_str());
  if (!enable)
    return;

  ros::Time start_time;

  if (system_time)
  {
    timeval time;
    gettimeofday(&time, NULL);
    start_time.sec = time.tv_sec;
    start_time.nsec = time.tv_usec * 1000;
  }
  else
    start_time = ros::Time::now();

  //assert the last timming is closed
  if (timing_data[key].size() > 0)
  {
    timing_entry& last_time_entry = timing_data[key].back();
    if (!last_time_entry.closed)
    {
      std::ostringstream stringStream;
      stringStream << "Bad use of the rtcus_timer. Started the timing of" << key.c_str()
          << "before closed the previous entry";
      ROS_ERROR("%s", stringStream.str().c_str());
      throw ros::Exception(stringStream.str());
    }
  }

  timing_entry entry;
  entry.start_time = start_time;
  timing_data[key].push_back(entry);
  //ROS_INFO("timing_data[%s].size()=%d",key.c_str(),timing_data.size());
}

ros::Duration Timer::stop(std::string key)
{

  if (!enable)
    return Duration(-1.0);

  ros::Time end_time;
  if (system_time)
  {
    timeval time;
    gettimeofday(&time, NULL);
    end_time.sec = time.tv_sec;
    end_time.nsec = time.tv_usec * 1000;
  }
  else
    end_time = ros::Time::now();

  if (timing_data[key].size() == 0)
  {
    std::ostringstream stringStream;
    stringStream << "Bad use of the rtcus_timer:" << key.c_str() << ". Stopped without start";
    ROS_ERROR("%s", stringStream.str().c_str());
    throw ros::Exception(stringStream.str());
  }
  else if (timing_data[key].size() > 0)
  {
    timing_entry& last_time_entry = timing_data[key].back();
    if (last_time_entry.closed)
    {
      std::ostringstream stringStream;
      stringStream << "Bad use of the rtcus_timer:" << key.c_str() << ". Stopped without start";
      ROS_ERROR("%s", stringStream.str().c_str());
      throw ros::Exception(stringStream.str());
    }
  }
  timing_entry& last_time_entry = timing_data[key].back();
  last_time_entry.end_time = end_time;
  last_time_entry.closed = true;
  return last_time_entry.getDuration();
  //ROS_INFO("closing timer stop %s",key.c_str());
}

ros::Duration Timer::last_duration(std::string key)
{
  if (!enable)
    return ros::Duration();

  //ROS_INFO("timer duration -> %s",key.c_str());
  if (timing_data[key].size() > 0)
  {
    timing_entry& last_time_entry = timing_data[key].back();
    return last_time_entry.getDuration();
  }
  else
  {
    std::ostringstream stringStream;
    stringStream << "Empty buffer: " << key.c_str() << ". Imposible to get the duration.";
    ROS_ERROR("%s", stringStream.str().c_str());
    throw ros::Exception(stringStream.str());
  }
}

void Timer::publish_data()
{
  if (!enable)
    return;

  for (std::map<std::string, std::list<timing_entry> >::iterator iter = timing_data.begin(); iter != timing_data.end();
      iter++)
  {
    std::list<timing_entry>& entries = iter->second;
    std::list<timing_entry>::iterator curr_entry = entries.begin();
    rtcus_timing::TimingEvent msg;
    msg.event_name = iter->first;
    std::string topic_name = msg.event_name;
    if (this->publishers.find(msg.event_name) == this->publishers.end())
    {
      this->publishers[topic_name] = this->nh_private.advertise < rtcus_timing::TimingEvent > (topic_name, 100);
    }

    while (curr_entry != entries.end())
    {
      timing_entry& entry = *curr_entry;
      msg.start_time = entry.start_time;
      msg.end_time = entry.end_time;
      msg.duration = entry.getDuration();
      event_pub_.publish(msg);
      this->publishers[topic_name].publish(msg);
      curr_entry = entries.erase(curr_entry);
    }
  }
}

void Timer::configuration_refresh()
{
  bool new_enable, new_system_time;
  bool clear = false;

  if (nh_.getParam("system_time", new_system_time) && system_time != new_system_time)
  {
    system_time = new_system_time;
    clear = true;
  }

  if (nh_.getParam("enable", new_enable) && enable != new_enable)
  {
    enable = new_enable;
    clear = true;
  }
  if (clear)
  {
    for (std::map<std::string, std::list<timing_entry> >::iterator iter = timing_data.begin();
        iter != timing_data.end(); iter++)
    {
      std::list<timing_entry>& entries = iter->second;
      entries.clear();
    }
    timing_data.clear();
  }
}
;

}
