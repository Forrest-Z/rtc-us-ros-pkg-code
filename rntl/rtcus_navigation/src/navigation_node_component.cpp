/*
 * NavigationComponent.h
 *
 *  Created on: Jul 4, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_navigation/navigation_component.h>

namespace rtcus_navigation
{

NavigationNodeComponent::NavigationNodeComponent(NavigationNodeComponent& covariant_component) :
    type_(covariant_component.type_), node_(covariant_component.node_), component_node_(
        covariant_component.component_node_), private_node_(covariant_component.component_node_), reset_service_(
        covariant_component.reset_service_), registered_(covariant_component.registered_)
{

}

NavigationNodeComponent::NavigationNodeComponent(NavComponentType type, std::string root_namespace) :
    type_(type), component_node_(root_namespace + "/" + getArchitectureComponentName()), private_node_(root_namespace), registered_(
        false)
{
  ROS_DEBUG( " * Creating component [%s//%s]", getArchitectureComponentName().c_str(), type_name().c_str());
}

bool NavigationNodeComponent::reset_cb()
{
  reset();
  return true;
}

std::string NavigationNodeComponent::getArchitectureComponentName() const
{
  return __navComponentsNames[type_];
}

NavComponentType NavigationNodeComponent::getComponentType() const
{
  return type_;
}

ros::NodeHandle& NavigationNodeComponent::getComponentNode()
{
  return component_node_;
}

ros::NodeHandle& NavigationNodeComponent::getPrivateNode()
{
  return private_node_;
}

void NavigationNodeComponent::unload()
{
  //reset_service_.shutdown();
  ROS_INFO( " * Unloading component of type [%s//%s]", __navComponentsNames[type_].c_str(), type_name().c_str());
  reset_service_.shutdown();
}

std::string NavigationNodeComponent::type_name() const
{
  return boost::units::detail::demangle(typeid(*this).name());

}

NavigationNodeComponent::~NavigationNodeComponent()
{
  ROS_INFO( " * Removing component of type [%s//%s]", __navComponentsNames[type_].c_str(), type_name().c_str());
  reset_service_.shutdown();
}

void NavigationNodeComponent::register_meta_info()
{
  if (!registered_)
  {
    ROS_INFO(
        " * Registering Navigation Component of type [%s//%s]", __navComponentsNames[type_].c_str(), type_name().c_str());
    ros::NodeHandle& n = this->component_node_;
    this->reset_service_ = n.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>(
        "reset", boost::bind(&NavigationNodeComponent::reset_cb, this));
    component_node_.setParam("component_name", this->type_name());
    registered_ = true;
  }
}

}

