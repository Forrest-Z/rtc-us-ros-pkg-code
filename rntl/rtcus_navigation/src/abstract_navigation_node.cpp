/*
 * abstract_navigation_node.cpp
 *
 *  Created on: Jan 20, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 */

#include <rtcus_navigation/abstract_navigation_node.h>

namespace rtcus_navigation
{

AbstractNavigationNode::AbstractNavigationNode() :
    NavigationNodeComponent::NavigationNodeComponent(tNavigationNode)
{
  this->config_.planner_frequency = 50;
}

AbstractNavigationNode::~AbstractNavigationNode()
{
  //be carefull with the double mutex
  if (this->control_timer_.isValid())
    this->control_timer_.stop();
  this->onStop();
}

void AbstractNavigationNode::registerComponent(shared_ptr<NavigationNodeComponent> component)
{
  if (components_.count(component->getComponentType()) != 0)
    components_[component->getComponentType()]->unload();

  //shared_ptr<NavigationNodeComponent> old_component = components_[component->getComponentType()];
  components_[component->getComponentType()] = component;
  component->register_meta_info();

  //if (old_component)
  //old_component->unload();
}

void AbstractNavigationNode::start()
{
  boost::mutex::scoped_lock l(m_mutex);
  this->control_timer_.start();
  this->onStart();
}

void AbstractNavigationNode::stop()
{
  boost::mutex::scoped_lock l(m_mutex);
  this->control_timer_.stop();
  this->onStop();
}

void AbstractNavigationNode::reset()
{
  this->config_.planner_frequency = 10.0;

  this->config_.reference_frame = "odom";
  this->config_.base_link_frame = "base_link";

  this->config_.prediction_base_link_frame = "base_link_prediction";
  this->control_timer_.stop();

  this->onReset();
  this->updateParameters();

  std::map<rtcus_navigation::NavComponentType, shared_ptr<rtcus_navigation::NavigationNodeComponent> >::iterator iter;
  ROS_INFO(" * Reseting navigation node...");
  ROS_INFO(" * Reseting subcomponents...");
  for (iter = this->components_.begin(); iter != components_.end(); iter++)
  {
    iter->second->reset();
    ROS_INFO("    |- %s", iter->second->type_name().c_str());
  }

  if (!defer_start_)
    this->control_timer_.start();
}

void AbstractNavigationNode::init(bool defer_start)
{
  ROS_INFO("== Initializing Navigation Node ==");
  ros::NodeHandle& nh = this->node_;
  ROS_INFO(" * Creating the Navigation Node Configure Server ");
  dynamic_reconfigure::Server<NavigationArchitectureConfig>::CallbackType f;
  f = boost::bind(&AbstractNavigationNode::configure_server_callback, this, _1);
  nav_architecture_configure_server_.setCallback(f);
  this->defer_start_ = defer_start;
  this->updateParameters();
  ROS_INFO(" * Using 'global_frame': %s", this->getReferenceFrame().c_str());
  ROS_INFO(" * Using 'base_link_frame': %s", this->getBaseFrameName().c_str());
  ROS_INFO(" * Defining the 'prediction_base_link_frame': %s", this->getBasePredictionFrame().c_str());

  //initialize components of node spetialization
  this->onInit();

  //---------- starting execution -------------------
  ROS_INFO("=== Navigation Architecture fully initializated: Starting ===");
  ROS_INFO(" * Creating the navigation control periodic task...");
  this->control_timer_ = nh.createTimer(ros::Duration(this->getPlannerFrequency().expectedCycleTime()),
                                        bind(&AbstractNavigationNode::controlTask, this, _1));
  if (!defer_start_)
  {
    ROS_INFO(
        "Starting node execution with a frequency of %lf Hz", 1.0/this->getPlannerFrequency().expectedCycleTime().toSec());
    this->control_timer_.start();
  }
  else
  {
    ROS_WARN("Deferred start activated. WAITING FOR THE START COMMAND.");
  }
}

void AbstractNavigationNode::updateParameters()
{
  ROS_INFO("Navigation Node. Updating static parameters from parameter server");
  //TODO: Move to the dynamic reconfigure sever
  if (!this->private_node_.getParam("global_frame", this->config_.reference_frame))
    this->private_node_.setParam("global_frame", this->config_.reference_frame);

  if (!this->private_node_.getParam("base_link_frame", this->config_.base_link_frame))
    this->private_node_.setParam("base_link_frame", this->config_.base_link_frame);

  if (!this->private_node_.getParam("prediction_base_link_frame", this->config_.prediction_base_link_frame))
    this->private_node_.setParam("prediction_base_link_frame", this->config_.prediction_base_link_frame);

  if (!this->private_node_.getParam("defer_start", this->defer_start_))
    this->private_node_.setParam("defer_start", defer_start_);

  double freq;
  if (!private_node_.getParam("planner_frequency", freq))
    this->private_node_.setParam("planner_frequency", this->config_.planner_frequency);
  else if (freq > 0.0)
  {
    this->config_.planner_frequency = freq;
    this->control_timer_.setPeriod(ros::Duration(1.0 / this->config_.planner_frequency));
  }
}

ros::Rate AbstractNavigationNode::getPlannerFrequency() const
{
  return this->config_.planner_frequency;
}

void AbstractNavigationNode::setPlannerFrequency(ros::Rate value)
{
  this->config_.planner_frequency = 1.0 / value.expectedCycleTime().toSec();
}

const NavigationArchitectureConfig& AbstractNavigationNode::configure_server_callback(
    NavigationArchitectureConfig &config)
{
  boost::mutex::scoped_lock l(m_mutex);
  ROS_INFO("== Executing configuration request ==");

  //TODO: A deeper configuration parameters constraint check.
  double freq = this->config_.planner_frequency;
  this->config_.planner_frequency = config.planner_frequency;
  if (freq > 0)
  {
    this->control_timer_.setPeriod(ros::Duration(1.0 / this->config_.planner_frequency));
  }
  else
  {
    config.planner_frequency = freq;
    ROS_WARN(" * Incorrect update data (planner frequency): %lf Hz. It has to be greater than 0.0", freq);
  }
  this->onConfigurationUpdate(config);

  this->config_ = config;
  this->onConfigurationUpdated(*this);
  ROS_INFO(" * node configuration request done.");
  return this->config_;
}

void AbstractNavigationNode::onInit()
{
}
void AbstractNavigationNode::onStart()
{
}
void AbstractNavigationNode::onStop()
{
}
void AbstractNavigationNode::onReset()
{
}
}
