/*
 * simple_dwa_ros.cpp
 *
 *  Created on: 22/12/2011
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_dwa/simple_dwa_ros.h>
#include <rtcus_navigation/navigation_node.h>
#include <rtcus_navigation/impl/navigation_node.h>
//#include <rtcus_dwa/visual_representation/dwa_algorithm_representation.h>

namespace rtcus_dwa
{

template<typename GoalType>
  DwaLocalPlannerBase<GoalType>::DwaLocalPlannerBase() :
      configure_server_(this->component_node_)

  {

  }

template<typename GoalType>
  DwaLocalPlannerBase<GoalType>::~DwaLocalPlannerBase()
  {
  }

template<typename GoalType>
  void DwaLocalPlannerBase<GoalType>::init(TNavigationNode& host_node_)

  {
    Dwa2DAlgorithm<GoalType, Twist2D>::init(this->getComponentNode());
    this->host_node_ = shared_ptr<TNavigationNode>(&host_node_);
    ROS_INFO("Creating DWA local planner");

    //----------- PLANNER DYNAMIC RECONFIGURE -----------
    ROS_INFO("DWA Dynamic reconfigure server created");
    dynamic_reconfigure::Server<SimpleDwaConfig>::CallbackType f;
    f = boost::bind(&DwaLocalPlannerBase<GoalType>::updatePlannerConfiguration, this, _1, _2);
    configure_server_.setCallback(f);

    //------------- NODE UPDATES --------------------------
    this->host_node_->onConfigurationUpdated.connect(
        boost::bind(&DwaLocalPlannerBase<GoalType>::onNavigationNodeConfigUpdated, this));

    //---------- ROBOT KINODYNAMICS ---------------------------
    this->host_node_->getKinodynamicModel()->onKinodynamicsChanged.connect(
        boost::bind(&DwaLocalPlannerBase<GoalType>::onRobotKinodynamicsUpdated, this, _1));
    NonHolonomicKinoDynamicsConfig kinodynamics;
    this->host_node_->getKinodynamicModel()->getKinodynamics(kinodynamics);
    //set initial kinodynamic
    this->onRobotKinodynamicsUpdated(kinodynamics);

    //---------- ROBOT SHAPE ---------------------------
    this->host_node_->getRobotShapeModel()->onShapeChanged.connect(
        boost::bind(&DwaLocalPlannerBase<GoalType>::onRobotShapeUpdated, this, _1));
    rtcus_robot_shapes::PolygonalRobot shape;
    this->host_node_->getRobotShapeModel()->getRobotShape(shape);
    //set initial robot shape
    this->onRobotShapeUpdated(shape);

    //------------ VISUAL REPRESENTATION-------------------------
    const DwaConfig& current_config = this->getConfig();
    ROS_INFO(
        "Setting the dwa heading simulation time step at the default control node period %f secs", current_config.simulation_time_step);

    ROS_INFO(
        "Building Visual representation for the DWA Node, representation frame (%s)", host_node_.getBasePredictionFrame().c_str());

    this->visual_representation_ = shared_ptr<rtcus_dwa::visual_representation::DwaVisualRepresentation<GoalType> >(
        new rtcus_dwa::visual_representation::DwaVisualRepresentation<GoalType>(
            *this, host_node_.getBasePredictionFrame(), host_node_.getReferenceFrame(),
            ros::NodeHandle(this->component_node_.getNamespace() + "/visual_representation")));

    //------------------------
    this->reset();
  }

template<typename GoalType>
  void DwaLocalPlannerBase<GoalType>::reset()
  {
    boost::mutex::scoped_lock l(m_mutex_);
    SimpleDwaConfig newconfig;
    newconfig.__fromServer__(this->component_node_);
    this->configure_server_.updateConfig(newconfig);
    this->updateConfig(newconfig);

    SimpleDwaConfig copyConfig = (SimpleDwaConfig&)this->getConfig();
    copyConfig.simulation_time_step = this->host_node_->getPlannerFrequency().expectedCycleTime().toSec();
    this->updateConfig(copyConfig);
    Dwa2DAlgorithm<GoalType, Twist2D>::reset();

  }

;
template<typename GoalType>
  void DwaLocalPlannerBase<GoalType>::onNavigationNodeConfigUpdated()
  {
    boost::mutex::scoped_lock l(m_mutex_);
    //frequency may have changed. Then we have to reconfigure using the current existing configuration and refresh.
    SimpleDwaConfig copyConfig = (SimpleDwaConfig&)this->getConfig();
    this->updatePlannerConfiguration(copyConfig, 1);
  }

template<typename GoalType>
  void DwaLocalPlannerBase<GoalType>::onRobotShapeUpdated(const PolygonalRobot& shape)
  {
    boost::mutex::scoped_lock l(m_mutex_);
    ROS_INFO(
        "ROSDWA SHAPE UPDATED; requested: %lf applied radius: %lf", shape.getRadius(), this->config_.getRobotShape().getRadius());
    this->config_.setRobotShape(shape);

    //Take the current existing configuration and refresh.
    updatePlannerConfiguration(this->config_, 1);

  }

template<typename GoalType>
  void DwaLocalPlannerBase<GoalType>::onRobotKinodynamicsUpdated(
      const rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig& kino_config)
  {
    boost::mutex::scoped_lock l(m_mutex_);
    this->config_.setKinoDynamicParameters(kino_config);
    //Take the current existing configuration and refresh.
    this->updatePlannerConfiguration(this->config_, 1);
    ROS_INFO("KINEMATICS UPDATED;");
  }

template<typename GoalType>
  const SimpleDwaConfig& DwaLocalPlannerBase<GoalType>::updatePlannerConfiguration(SimpleDwaConfig &config,
                                                                                   uint32_t level)
  {
    {
      m_mutex_.try_lock();
      if (this->host_node_ != NULL && config.auto_simulation_time_step)
        config.simulation_time_step = this->host_node_->getPlannerFrequency().expectedCycleTime().toSec();

      this->updateConfig(config);
      m_mutex_.unlock();
    }

    //REFRESH THE REPRESENTATION
    if (this->preconditions())
    {
      Twist2D resulting_action;

      const TTaskStatus& current_status = this->host_node_->getCurrentTaskStatus();
      this->computeVelocityCommands(*(current_status.getComputedObstacles()), *(current_status.getComputedGoal()),
                                    *(current_status.getStateEstimation()), resulting_action);
    }
    this->onConfigUpdated(*this);
    return this->getConfig();
  }

template<typename GoalType>
  bool DwaLocalPlannerBase<GoalType>::computeVelocityCommands(const pcl::PointCloud<pcl::PointXY>& obstacles,
                                                              const GoalType& goal, const DynamicState2D& local_state,
                                                              Twist2D& resulting_action)
  {
    boost::mutex::scoped_lock l(m_mutex_);
    rtcus_dwa::Twist2D best_cmd;
    bool action_found = false;
    this->onPrecomputeCommand(*this);
    {
      //send data to the planner
      this->setObstacles(obstacles);
      this->setGoal(goal);
      this->setState(local_state.twist, this->host_node_->getPlannerFrequency().expectedCycleTime().toSec());
      //clear result values
      resulting_action.linear = 0.0;
      resulting_action.lateral = 0.0;
      resulting_action.angular = 0.0;

      action_found = Dwa2DAlgorithm<GoalType, Twist2D>::computeVelocityCommands(best_cmd);
      resulting_action = best_cmd;
    }
    this->onPostComputeCommand(*this, resulting_action, local_state, goal);
    return action_found;
  }

//spetialization for dwa
template class DwaLocalPlannerBase<rtcus_dwa::PointXY> ;

//spetialization for shared dwa
template class DwaLocalPlannerBase<rtcus_dwa::Twist2D> ;

}

