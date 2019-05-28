/*
 * timming_observer.h
 *
 *  Created on: Jun 19, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef TIMMING_OBSERVER_H_
#define TIMMING_OBSERVER_H_

#include <rtcus_navigation/navigation_node.h>
#include <rtcus_timing/timer.h>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <std_msgs/Duration.h>
#include <std_msgs/Float32.h>

namespace rtcus_navigation_tools
{
using namespace rtcus_navigation;
using namespace boost;

template<typename NavigationNodeType>
  class TimeObserver
  {

  protected:
    rtcus_timing::Timer timer_;
    shared_ptr<NavigationNodeType> node_;
    ros::Publisher execution_delay_pub_;
    ros::Publisher current_execution_rate_pub_;

    void navigation_control_task_start(const NavigationNodeType& nn, const ros::Time& application_time,
                                       const ros::TimerEvent& time)
    {
      ros::Duration delay = time.current_real - time.current_expected;
      ROS_INFO(
          "=====  Starting Control Task - Application time [%lf sec]. Control Loop at [%lf]... delayed [%lf] =====", application_time.toSec(), time.current_real.toSec(), delay.toSec());
      std_msgs::Duration delay_msg;
      delay_msg.data = delay;
      execution_delay_pub_.publish(delay_msg);

      std_msgs::Float32 current_rate;
      current_rate.data = 1.0 / (time.current_real.toSec() - time.last_real.toSec());
      this->current_execution_rate_pub_.publish(current_rate);

    }

    void navigation_event_stop(const NavigationNodeType& nn, const char* msg, const char* key)
    {
      ros::Duration duration = timer_.stop(key);
      //ROS_INFO(
      //    "Timming [%s] -> %lf ms [%lf percent]", key, duration.toSec()*1000, duration.toSec()/nn.getPlannerFrequency().expectedCycleTime().toSec());
    }
    void navigation_event_start(const NavigationNodeType& nn, const char* msg, const char* key)
    {
      //ROS_INFO("Timmig Start [%s]", key);
      //ROS_INFO("\tNumber of obstacles %ld", nn.getWorldPerceptionPort()->getLastUpdateWorld().getConstData().size());

      /*if (nn.getActionPort()->getActionHistory().size() > 0)
       ROS_INFO("\tLast command: %lf, %lf", nn.getActionPort()->getActionHistory().back().linear,
       nn.getActionPort()->getActionHistory().back().angular);*/

      timer_.start(key);
    }

  public:
    TimeObserver(const shared_ptr<NavigationNodeType>& node) :
        timer_("~", true), node_(node)
    {
      ros::NodeHandle nh_("~/time_observer");
      execution_delay_pub_ = nh_.advertise<std_msgs::Duration>("task_event_delay", 10);
      current_execution_rate_pub_ = nh_.advertise<std_msgs::Float32>("current_execution_rate", 10);
      node->onControlTaskBegin.connect(bind(&TimeObserver::navigation_event_start, this, _1, "", "control_task"));
      node->onControlTaskBegin.connect(bind(&TimeObserver::navigation_control_task_start, this, _1, _2, _3));
      node->onControlTaskEnd.connect(bind(&TimeObserver::navigation_event_stop, this, _1, "", "control_task"));
      node->onControlTaskEnd.connect(bind(&TimeObserver::navigation_event_flush, this));

      node->onProcessingLocalWorldBegin.connect(
          bind(&TimeObserver::navigation_event_start, this, _1, "", "world_proc"));
      node->onProcessingLocalWorldEnd.connect(bind(&TimeObserver::navigation_event_stop, this, _1, "", "world_proc"));

      node->onProcessingLocalGoalBegin.connect(bind(&TimeObserver::navigation_event_start, this, _1, "", "goal_proc"));
      node->onProcessingLocalGoalEnd.connect(bind(&TimeObserver::navigation_event_stop, this, _1, "", "goal_proc"));

      node->onPlanningBegin.connect(bind(&TimeObserver::navigation_event_start, this, _1, "", "planning"));
      node->onPlaningEnd.connect(bind(&TimeObserver::navigation_event_stop, this, _1, "", "planning"));

      node->onStateEstimationBegin.connect(bind(&TimeObserver::navigation_event_start, this, _1, "", "estimation"));
      node->onStateEstimationEnd.connect(bind(&TimeObserver::navigation_event_stop, this, _1, "", "estimation"));

      node->onStateCorrectionBegin.connect(bind(&TimeObserver::navigation_event_start, this, _1, "", "correction"));
      node->onStateCorrectionEnd.connect(bind(&TimeObserver::navigation_event_stop, this, _1, "", "correction"));
    }

    void navigation_event_flush()
    {
      this->timer_.publish_data();
    }
  };

}

#endif /* TIMMING_OBSERVER_H_ */
