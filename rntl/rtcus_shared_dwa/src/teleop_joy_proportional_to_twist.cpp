#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <ros/duration.h>
#include <boost/thread/mutex.hpp>
#include <std_msgs/String.h>
#include <rtcus_stamp/stamped.h>
#include <boost/bind.hpp>
#include <rtcus_shared_dwa/ProportionalJoyTeleopConfig.h>
#include <dynamic_reconfigure/server.h>

using namespace rtcus_stamp;
using namespace geometry_msgs;

class TeleopJoyTwist
{
public:
  TeleopJoyTwist();
  void publish_msgs();

private:
  void onControlCommandCallback(const boost::shared_ptr<const std_msgs::String>& cmd);
  const rtcus_shared_dwa::ProportionalJoyTeleopConfig& configure_server_callback(
      rtcus_shared_dwa::ProportionalJoyTeleopConfig &config);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  rtcus_shared_dwa::ProportionalJoyTeleopConfig config_;
  ros::Publisher vel_pub_;
  //ros::Publisher goal_pub_;

  ros::Subscriber joy_sub_;
  ros::Subscriber control_sub_;

  sensor_msgs::Joy last_command;
  bool send_continous_;
  geometry_msgs::Twist cmd_vel_;
  boost::mutex m_mutex_;
  bool running_;
  dynamic_reconfigure::Server<rtcus_shared_dwa::ProportionalJoyTeleopConfig> dynamic_reconfigure_;
};

TeleopJoyTwist::TeleopJoyTwist() :
    private_nh_("~"), send_continous_(true), running_(true)
{
  this->config_.axis_linear = 1;
  this->config_.axis_angular = 0;
  this->config_.scale_linear = 1;
  this->config_.scale_angular = 1;
  this->config_.robot_base_link = "base_link";

  if (!private_nh_.getParam("axis_linear", config_.axis_linear))
    private_nh_.setParam("axis_linear", config_.axis_linear);

  if (!private_nh_.getParam("axis_angular", config_.axis_angular))
    private_nh_.setParam("axis_angular", config_.axis_angular);

  if (!private_nh_.getParam("scale_linear", config_.scale_linear))
    private_nh_.setParam("scale_linear", config_.scale_linear);

  if (!private_nh_.getParam("scale_angular", config_.scale_angular))
    private_nh_.setParam("scale_angular", config_.scale_angular);

  if (!private_nh_.getParam("robot_base_link", config_.robot_base_link))
    private_nh_.setParam("robot_base_link", config_.robot_base_link);

  bool defer_start;
  if (!private_nh_.getParam("defer_start", defer_start))
    private_nh_.setParam("defer_start", !running_);
  else
    this->running_ = !defer_start;

  dynamic_reconfigure::Server<rtcus_shared_dwa::ProportionalJoyTeleopConfig>::CallbackType f;
  f = boost::bind(&TeleopJoyTwist::configure_server_callback, this, _1);
  this->dynamic_reconfigure_.setCallback(f);

  control_sub_ = private_nh_.subscribe<std_msgs::String>(
      "control", 10, boost::bind(&TeleopJoyTwist::onControlCommandCallback, this, _1));

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  //goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("joy_goal", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, boost::bind(&TeleopJoyTwist::joyCallback, this, _1));
  cmd_vel_.linear.x = cmd_vel_.linear.y = cmd_vel_.linear.z = 0;
  cmd_vel_.angular.x = cmd_vel_.angular.y = cmd_vel_.angular.z = 0;
}

const rtcus_shared_dwa::ProportionalJoyTeleopConfig& TeleopJoyTwist::configure_server_callback(
    rtcus_shared_dwa::ProportionalJoyTeleopConfig &config)
{
  return this->config_;
}

void TeleopJoyTwist::onControlCommandCallback(const boost::shared_ptr<const std_msgs::String>& cmd)
{
  boost::mutex::scoped_lock lock(m_mutex_);
  if (cmd->data == "STOP")
    this->running_ = false;
  else if (cmd->data == "START")
    this->running_ = true;
  else
    ROS_ERROR("Remote manual teleop. Incorrect control command '%s' ", cmd->data.c_str());

}

void TeleopJoyTwist::publish_msgs()
{
  boost::mutex::scoped_lock lock(m_mutex_);
  if (running_)
  {

    vel_pub_.publish(cmd_vel_);

    //geometry_msgs::PoseStamped goal;
    //goal_pub_.publish(goal);
  }
}

void TeleopJoyTwist::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  {
    boost::mutex::scoped_lock lock(m_mutex_);
    last_command = *joy;
    cmd_vel_.angular.z = last_command.axes[config_.axis_angular] * config_.scale_angular;
    cmd_vel_.linear.x = last_command.axes[config_.axis_linear] * config_.scale_linear;
  }
  this->publish_msgs();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_joy");
  TeleopJoyTwist teleop_turtle;

  ros::Rate rate(20);
  while (ros::ok())
  {
    teleop_turtle.publish_msgs();
    rate.sleep();
    ros::spinOnce();
  }
}
