/*
 * abstract_navigation_node.h
 *
 *  Created on: Jan 20, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef ABSTRACT_NAVIGATION_NODE_H_
#define ABSTRACT_NAVIGATION_NODE_H_

#include <rtcus_navigation/navigation_component.h>
#include <rtcus_navigation/NavigationArchitectureConfig.h>
#include <dynamic_reconfigure/server.h>
#include <map>

namespace rtcus_navigation
{

using namespace std;
using namespace rtcus_navigation;
using namespace boost;
using namespace rtcus_stamp;

/**
 * \brief non-typed navigation node. It is useful to set soft-constrained types dependencies.
 * */
class AbstractNavigationNode : public NavigationNodeComponent
{

public:
  void init(bool defer_start = true);
  void start();
  void stop();
  void reset();

  boost::signal<void(const AbstractNavigationNode& sender)> onConfigurationUpdated;

  //--------------------------------------------------------------------
  /**
   * \brief soft typed component dependency.
   * */
  template<typename TComponent>
    shared_ptr<TComponent> getComponent(NavComponentType type)
    {
      return dynamic_pointer_cast<TComponent>(components_[type]);
    }

  inline const NavigationArchitectureConfig& getConfig() const
  {
    return config_;
  }
  /*! \ brief returns the reference frame of the current robot state estimation
   * */
  inline std::string getReferenceFrame() const
  {
    return this->config_.reference_frame;
  }

  inline std::string getBasePredictionFrame() const
  {
    return this->config_.prediction_base_link_frame;
  }

  inline std::string getBaseFrameName() const
  {
    return this->config_.base_link_frame;
  }

  void setPlannerFrequency(ros::Rate value);
  void updateParameters();
  ros::Rate getPlannerFrequency() const;

  //------------------------------------------------------------
protected:
  AbstractNavigationNode();
  virtual ~AbstractNavigationNode();
  map<NavComponentType, shared_ptr<NavigationNodeComponent> > components_;

  NavigationArchitectureConfig config_;
  ros::Timer control_timer_;
  boost::mutex m_mutex;
  void registerComponent(shared_ptr<NavigationNodeComponent> component);
  bool defer_start_;
  dynamic_reconfigure::Server<rtcus_navigation::NavigationArchitectureConfig> nav_architecture_configure_server_;
  const NavigationArchitectureConfig& configure_server_callback(NavigationArchitectureConfig &config);

  //methods to be overriden
  virtual void onConfigurationUpdate(NavigationArchitectureConfig &config)=0;
  virtual void onInit();
  virtual void onStart();
  virtual void onStop();
  virtual void onReset();
  virtual void controlTask(const ros::TimerEvent time)=0;

};

}

#endif /* ABSTRACT_NAVIGATION_NODE_H_ */
