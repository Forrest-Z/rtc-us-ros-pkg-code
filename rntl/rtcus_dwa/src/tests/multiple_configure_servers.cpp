#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <gtest/gtest.h>
#include <dynamic_reconfigure/server.h>
#include <rtcus_dwa/SimpleDwaConfig.h>

using namespace tf;
using namespace rtcus_dwa;

void configure_server_callback(SimpleDwaConfig &config, uint32_t level)
{
  ROS_INFO("oook");
}

TEST(multiple_configuration_server,basic)
{/*

 dynamic_reconfigure::Server<SimpleDwaConfig> server;
 dynamic_reconfigure::Server<SimpleDwaConfig>::CallbackType f;
 f = boost::bind(&configure_server_callback, _1, _2);
 server.setCallback(f);
 for (int i = 0; i < 10; i++)
 {
 ros::spinOnce();
 ros::Duration(1).sleep();
 }*/
}
