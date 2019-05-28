/*
 * simulation_motion_model_prediction_error_watch.h
 *
 *  Created on: Aug 21, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef SIMULATION_MOTION_MODEL_PREDICTION_ERROR_WATCH_H_
#define SIMULATION_MOTION_MODEL_PREDICTION_ERROR_WATCH_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <rtcus_nav_msgs/DynamicState2D.h>
#include <rtcus_stamp/stamped.h>
#include <boost/lambda/bind.hpp>
#include <rtcus_navigation/navigation_node.h>
#include <rtcus_conversions/conversions.h>
#include <std_msgs/Float64.h>

namespace rtcus_navigation_tools
{
using namespace rtcus_navigation;
using namespace rtcus_stamp;
using namespace rtcus_nav_msgs;

/*/brief this class subscribes to the real state, the odometry state and the real predictive motion state estimation
 * */
template<typename NavigationNodeType>
  class SimulatorPredictionMotionModelErrorWatch
  {
  public:
    USING_NAVIGATION_TYPES(NavigationNodeType)
    ;
    typedef typename NavigationNodeType::TActionType ActionType;
    typedef typename NavigationNodeType::TGoalType GoalType;

    StampedData<DynamicState2D> last_real_position_;
    StampedData<DynamicState2D> state_estimation_;
    StampedData<DynamicState2D> last_state_reading_;
    StampedData<Twist2D> last_action;
    ros::Time reception_time_;

    ros::Subscriber base_ground_truth_sub_;
    ros::Publisher estimation_error_;

    void show_error(const TTaskStatus&status)
    {
      ROS_INFO("=== STATE ANALYSIS ===");
      ROS_INFO(
          "|- Planning time: %lf | expected application time: %lf", status.getPlanningTime().toSec(), status.getExpectedApplicationTime().toSec());
      ROS_INFO(
          " |-Ground Truth received at time [%lf] and stamped with [%lf]:", ros::Time::now().toSec(), last_real_position_.getStamp().toSec());
      ROS_INFO(
          " |   |- Pose [%lf %lf %lf] .", last_real_position_->pose.x, last_real_position_->pose.y, last_real_position_->pose.phi);
      ROS_INFO(
          " |   |- Twist [linear %lf angular %lf]", last_real_position_->twist.linear, last_real_position_->twist.angular);

      ROS_INFO(
          " |- Sensor State reading at [%lf] received at (%lf) ", last_state_reading_.getStamp().toSec(), reception_time_.toSec());
      ROS_INFO(
          " |   |- Pose [%lf %lf %lf] .", last_state_reading_->pose.x, last_state_reading_->pose.y, last_state_reading_->pose.phi);
      ROS_INFO(
          " |   |- Twist [linear %lf angular %lf]", last_state_reading_->twist.linear, last_state_reading_->twist.angular);

      ROS_INFO( " |- State Estimation at (%lf)", state_estimation_.getStamp().toSec());

      ROS_INFO(
          " |   |- Pose [%lf %lf %lf] .", state_estimation_->pose.x, state_estimation_->pose.y, state_estimation_->pose.phi);
      ROS_INFO(
          " |   |- Twist [linear %lf angular %lf]", state_estimation_->twist.linear, state_estimation_->twist.angular);

      //TODO: This does not happen never... it is needed a buffer of state estimations
      if (last_real_position_.getStamp() == state_estimation_.getStamp())
      {
        double err_x = last_real_position_->pose.x - state_estimation_->pose.x;
        double err_y = last_real_position_->pose.y - state_estimation_->pose.y;
        double err_phi = last_real_position_->pose.phi - state_estimation_->pose.phi;
        ROS_INFO("* Error x: %lf | error y: %lf | error phi: %lf", err_x, err_y, err_phi);
        std_msgs::Float64 msg;
        msg.data = sqrt(err_x * err_x + err_y * err_y + err_phi * err_phi);
        estimation_error_.publish(msg);
      }
      else
      {
        //  ROS_INFO(
        //      "state estimation non-comparable, estimation stamp [%lf], ground truth stamp [%lf]", state_estimation_.getStamp().toSec(), last_real_position_.getStamp().toSec());
      }
    }
    void onPoseGroundTruthCallback(const nav_msgs::Odometry::ConstPtr& real_pos)
    {
      tf::Quaternion orientation;
      tf::quaternionMsgToTF(real_pos->pose.pose.orientation, orientation);
      rtcus_conversions::Conversions::convert(real_pos->pose.pose, last_real_position_->pose);
      last_real_position_.setStamp(real_pos->header.stamp);
    }

    void onStateReadingReceived(const StatePort<DynamicState2D>& state_port, const ros::Time& reception_time)
    {
      state_port.copyLastState(last_state_reading_);
      reception_time_ = reception_time;

    }

    void onStateEstimation(const NavigationNodeType& nn)
    {
      nn.getStateEstimation()->getLastStateEstimation(this->state_estimation_);
      const TTaskStatus& status = nn.getCurrentTaskStatus();
      show_error(status);

    }

    void onPlanningResult(const NavigationNodeType& nn, const ActionType& last_action,
                          const ros::Time& application_time)
    {

    }

    void onProcessingLocalGoalEnd(const NavigationNodeType& nn, const GoalType& resulting_goal,
                                  const ros::Time& application_time)
    {
      //ROS_INFO_STREAM("=== GOAL ANALYSIS ===\n" << resulting_goal);
    }

    SimulatorPredictionMotionModelErrorWatch(NavigationNodeType& nn)
    {
      ros::NodeHandle nh("~");
      boost::function<void(const nav_msgs::Odometry::ConstPtr)> f = boost::bind(
          &SimulatorPredictionMotionModelErrorWatch::onPoseGroundTruthCallback, this, _1);

      base_ground_truth_sub_ = nh.subscribe<nav_msgs::Odometry>("/base_pose_ground_truth", 1, f);

      estimation_error_ = nh.advertise<std_msgs::Float64>("simulated_ground_truth_helper/estimation_error", 1);

      nn.onStateEstimationEnd.connect(
          bind(&SimulatorPredictionMotionModelErrorWatch<NavigationNodeType>::onStateEstimation, this, _1));

      nn.getStatePort()->onStateReceived.connect(
          bind(&SimulatorPredictionMotionModelErrorWatch<NavigationNodeType>::onStateReadingReceived, this, _1, _2));

      nn.onPlanningResult.connect(
          bind(&SimulatorPredictionMotionModelErrorWatch<NavigationNodeType>::onPlanningResult, this, _1, _2, _3));

      nn.onProcessingLocalGoalEnd.connect(
          bind(&SimulatorPredictionMotionModelErrorWatch<NavigationNodeType>::onProcessingLocalGoalEnd, this, _1, _2,
               _3));
    }

    nav_msgs::Odometry getLastRealPosition() const
    {
      return this->getLastRealPosition();
    }
  };
}

#endif /* SIMULATION_MOTION_MODEL_PREDICTION_ERROR_WATCH_H_ */
