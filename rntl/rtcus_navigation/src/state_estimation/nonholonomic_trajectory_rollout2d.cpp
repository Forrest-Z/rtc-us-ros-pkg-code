/*
 * nonholonomic_trajectory_rollout2d.cpp
 *
 *  Created on: Feb 7, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */
#include <rtcus_navigation/state_estimation/non_time_correction_state_estimation.h>
#include <rtcus_motion_models/motion_models/non_holonomic_trajectory_rollout_2d.h>
#include <rtcus_navigation/state_estimation.h>
#include <rtcus_navigation/state_estimation/default_state_estimation.h>
#include <rtcus_motion_models/motion_models/motion_models_impl.h>
#include <rtcus_motion_models/motion_models/non_holonomic_2d.h>
#include <rtcus_nav_msgs/DynamicState2D.h>
#include <rtcus_nav_msgs/Twist2D.h>
#include <rtcus_kinodynamic_description/NonHolonomicKinoDynamicsConfig.h>
#include <rtcus_navigation/kinodynamic_model.h>
#include <rtcus_navigation/kinodynamic_models/configurable_static_robot_kinodynamics.h>
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>

namespace rtcus_navigation
{

namespace state_estimation
{

using namespace rtcus_navigation::kinodynamic_models;
class nonholonomic_trajectory_rollout2d : public DefaultStateEstimation<DynamicState2D, Twist2D, ROSTimeModel>
{
protected:
  shared_ptr<rtcus_motion_models::TrajectoryRolloutNonHolonomic2D> internal_motion_model_;
  shared_ptr<KinoDynamicModel<rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig> > kino_model_;
  boost::signals::connection c1;
  boost::signals::connection c2;

public:
  nonholonomic_trajectory_rollout2d()
  {

  }
  virtual ~nonholonomic_trajectory_rollout2d()
  {
    c1.disconnect();
    c2.disconnect();
    RTCUS_ASSERT(!c1.connected() && !c2.connected());
  }

  virtual void unload()
  {
    DefaultStateEstimation<DynamicState2D, Twist2D>::unload();
    c1.block(true);
    c1.disconnect();
    c2.block(true);
    c2.disconnect();
    AbstractNavigationNode& cnn = const_cast<rtcus_navigation::AbstractNavigationNode&>(*this->host_node_);
    //TODO:CLEAN BEFOERE, THIS ONE WORKED FOR DISCONNECTION
    cnn.onConfigurationUpdated.disconnect(
        boost::bind(&nonholonomic_trajectory_rollout2d::onWorkingFrequencyUpdate, this, _1));
    this->kino_model_->onKinodynamicsChanged.disconnect(
        boost::bind(&nonholonomic_trajectory_rollout2d::onKinoDynamicUpdated, this, _1));

    RTCUS_ASSERT(!c1.connected() && !c2.connected());
  }

  virtual void init(const rtcus_navigation::AbstractNavigationNode& nn, const Twist2D& stopAction)
  {
    //SOME DIRTY TRICKS HERE. VIOLATING CONST... TODO: perhaps remove const constraint of the AbstractNavigationNode from NavComponent init methods to make posible event subscriptions.
    AbstractNavigationNode& cnn = const_cast<rtcus_navigation::AbstractNavigationNode&>(nn);

    bool use_custom_kinodynamics = false;
    if (!this->getComponentNode().getParam("trajectory_rollout_estimation/use_custom_kinodynamics",
                                           use_custom_kinodynamics))
    {
      this->getComponentNode().setParam("trajectory_rollout_estimation/use_custom_kinodynamics",
                                        use_custom_kinodynamics);
    }

    if (!use_custom_kinodynamics)
    {
      ROS_INFO("%s. Using navigation node kinodynamics specification.", getClassName(*this).c_str());
      kino_model_ = cnn.getComponent<KinoDynamicModel<rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig> >(
          tKinoDynamicModel);

    }
    else
    {
      std::string dynamic_reconfigure_uri = this->getComponentNode().getNamespace() + "/kinodynamic_model";
      ROS_INFO(
          "%s.  use_custom_kinodynamics:true -> A new kinodynamic specification at %s", getClassName(*this).c_str(), dynamic_reconfigure_uri.c_str());

      kino_model_ = shared_ptr<
          ConfigurableKinodynamicModel<rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig> >(
          new ConfigurableKinodynamicModel<rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig>(
              make_shared<DefaultKinodynamicModel<rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig> >(),
              ros::NodeHandle(dynamic_reconfigure_uri))

              );
      kino_model_->init();
    }

    rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig kinodesc;
    kino_model_->getKinodynamics(kinodesc);
    if (!internal_motion_model_)
    {
      internal_motion_model_ = shared_ptr<rtcus_motion_models::TrajectoryRolloutNonHolonomic2D>(
          new rtcus_motion_models::TrajectoryRolloutNonHolonomic2D(
              nn.getPlannerFrequency().expectedCycleTime().toSec() / 10.0, kinodesc));
      this->setMotionModel(internal_motion_model_);
    }
    //------------------------------------------------------
    this->c1 = kino_model_->onKinodynamicsChanged.connect(
        boost::bind(&nonholonomic_trajectory_rollout2d::onKinoDynamicUpdated, this, _1));
    this->onKinoDynamicUpdated(kinodesc);

    this->c2 = cnn.onConfigurationUpdated.connect(
        boost::bind(&nonholonomic_trajectory_rollout2d::onWorkingFrequencyUpdate, this, _1));
    this->onWorkingFrequencyUpdate(nn);

    DefaultStateEstimation<DynamicState2D, Twist2D, ROSTimeModel>::init(nn, stopAction);
  }

  void onKinoDynamicUpdated(const rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig& kinodesc)
  {
    if (c1.connected())
      this->internal_motion_model_->setKinodynamics(kinodesc);
  }

  void onWorkingFrequencyUpdate(const rtcus_navigation::AbstractNavigationNode& nn)
  {
    //TODO: Why do I have to do this?
    if (c2.connected())
    {
      //TODO: Take this as parameter. fifty times faster than the main navigation frequency loop
      this->internal_motion_model_->setTimeIntegrationPeriod(
          nn.getPlannerFrequency().expectedCycleTime().toSec() / 50.0);
      ROS_INFO(
          " * %s - Setting the simulation frequency period to %lf whle the working frequency period is %lf", getClassName(*this).c_str(), this->internal_motion_model_->getTimeIntegrationPeriod(), nn.getPlannerFrequency().expectedCycleTime().toSec());
    }
  }
};

PLUGINLIB_DECLARE_CLASS(rtcus_navigation, nonholonomic_trajectory_rollout2d,
    rtcus_navigation::state_estimation::nonholonomic_trajectory_rollout2d,
    rtcus_navigation::NavigationNodeComponent)

}}

