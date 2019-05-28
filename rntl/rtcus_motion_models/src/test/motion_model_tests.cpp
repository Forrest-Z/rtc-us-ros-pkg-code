/*
 * motion_model_tests.cpp
 *
 *  Created on: Apr 10, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <mrpt_bridge/pose_conversions.h>
#include <gtest/gtest.h>
#include <boost/foreach.hpp>

#include <rtcus_motion_models/motion_models.h>
#include <rtcus_motion_models/motion_models/non_holonomic_2d.h>
#include <rtcus_nav_msgs/Twist2D.h>
#include <rtcus_nav_msgs/DynamicState2D.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/tf.h>

using namespace rtcus_nav_msgs;
using namespace rtcus_motion_models;
using namespace geometry_msgs;

#include <gtest/gtest.h>

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
  ros::Duration d;

}

namespace rtcus_compositions
{
template<>
  void StateComposer::compose<DynamicState2D, DynamicState2D>(const DynamicState2D& state,
                                                              const DynamicState2D& transform, DynamicState2D& dst)
  {
    StateComposer::compose(state.pose, transform.pose, dst.pose);
    //Supposing the local_reference_frame is static (not moving)
    dst.twist = state.twist;
  }
}
//1- FINALSTATE SINGLE-ACTION NONSTAMPED N/A(SYNC)
TEST(MotionModels, predictState1_Final_state_Single_action_NonStamped_NA_SYNC)
{
  DeterministicNonHolonomic2D<Pose, Twist> motion_model;

  Pose initial_state;
  initial_state.orientation.w = 1.0;

  Twist cmd_vel;
  cmd_vel.linear.x = 1.0;
  ros::Duration dt(1.0);
  Pose final_state;

  motion_model.predictState(initial_state, cmd_vel, dt, final_state);
  EXPECT_DOUBLE_EQ(1.0,final_state.position.x) << "\ninitial_state\n" << initial_state << "\nfinal_state\n"
      << final_state;
  ASSERT_DOUBLE_EQ(final_state.position.y, 0.0);
  ASSERT_DOUBLE_EQ(tf::getYaw(final_state.orientation), 0.0);

  cmd_vel.angular.z = 0.1;
  motion_model.predictState(initial_state, cmd_vel, dt, final_state);
  ROS_INFO_STREAM("final state:"<< final_state);
  //because the second action moves the robot from the straight line. then:
  EXPECT_TRUE(final_state.position.y!=0) << final_state;
  EXPECT_LE(final_state.position.x,1.0) << final_state;

}

//1b- FINALSTATE SINGLE-ACTION NONSTAMPED N/A(SYNC)
TEST(MotionModels, predictState1b_Final_state_Single_action_NonStamped_NA_SYNC)
{
  DeterministicNonHolonomic2D<mrpt::poses::CPose2D, rtcus_nav_msgs::Twist2D> motion_model;

  mrpt::poses::CPose2D initial_state;
  initial_state.phi(0);

  rtcus_nav_msgs::Twist2D cmd_vel;
  cmd_vel.linear = 1.0;
  ros::Duration dt(1.0);
  mrpt::poses::CPose2D final_state;

  motion_model.predictState(initial_state, cmd_vel, dt, final_state);
  EXPECT_DOUBLE_EQ(1.0,final_state.x()) << "\ninitial_state\n" << initial_state << "\nfinal_state\n" << final_state;

  ROS_INFO("final state is %lf %lf %lf", final_state.x(), final_state.y(), final_state.phi());
  cmd_vel.angular = 1.0;
  motion_model.predictState(initial_state, cmd_vel, dt, final_state);

  //because the second action moves the robot from the straight line. then:
  EXPECT_TRUE(final_state.y()!=0);

}

//2- FINALSTATE SINGLE-ACTION NONSTAMPED N/A(SYNC)
TEST(MotionModels, predictState2_Final_state_Single_action_Stamped_NA_SYNC)
{
  DeterministicNonHolonomic2D<Pose, Twist> motion_model;

  ros::Time start_time(0.0);
  StampedData<Pose> initial_state(start_time, "odom");
  initial_state->orientation.w = 1.0;

  Twist cmd_vel;
  cmd_vel.linear.x = 1.0;
  ros::Duration dt(1.0);
  StampedData<Pose> final_state;

  motion_model.predictState(initial_state, cmd_vel, dt, final_state);
  ROS_INFO_STREAM("final state is "<<*final_state<<" and stamp is"<< final_state.getStamp().toSec());
  EXPECT_DOUBLE_EQ(1.0,final_state->position.x) << "\ninitial_state\n" << *initial_state << "\nfinal_state\n"
      << *final_state;

  EXPECT_TRUE((start_time+dt)==final_state.getStamp()) << final_state.getStamp().toSec();
  EXPECT_TRUE(initial_state.getFrameId()==final_state.getFrameId());
}

//3- FINALSTATE MLTIPLEACTIONS NONSTAMPED SYNC
TEST(MotionModels, predictState3_Final_state_MultipleActions_NonStamped_SYNC)
{
  DeterministicNonHolonomic2D<Pose, Twist> motion_model;

  ros::Time start_time(0.0);
  Pose initial_state;
  initial_state.orientation.w = 1.0;

  vector<Twist> actions;
  Twist cmd_vel;
  cmd_vel.linear.x = 1.0;
  actions.push_back(cmd_vel);
  cmd_vel.angular.z = 0.1;
  actions.push_back(cmd_vel);

  ros::Duration dt(1.0);
  Pose final_state;
  motion_model.predictState(initial_state, actions, dt, final_state);

  //because the second action moves the robot from the straight line. then:
  EXPECT_TRUE(final_state.position.y!=0);
  EXPECT_LE(final_state.position.x, 2.0);
}

//4- FINALSTATE MLTIPLEACTIONS STAMPED SYNC
TEST(MotionModels, predictState3_Final_state_MultipleActions_Stamped_SYNC)
{
  DeterministicNonHolonomic2D<Pose, Twist> motion_model;

  ros::Time start_time(0.0);
  StampedData<Pose> initial_state(ros::Time(0), "");
  initial_state->orientation.w = 1.0;

  vector<Twist> actions;
  Twist cmd_vel;
  cmd_vel.linear.x = 1.0;
  actions.push_back(cmd_vel);
  cmd_vel.angular.z = 1.0;
  actions.push_back(cmd_vel);

  ros::Duration dt(10.0);
  StampedData<Pose> final_state;
  motion_model.predictState(initial_state, actions, dt, final_state);

  //because the second action moves the robot from the straight line. then:
  EXPECT_TRUE(final_state->position.y!=0);
  EXPECT_TRUE(initial_state.getFrameId()==final_state.getFrameId());
  EXPECT_DOUBLE_EQ(final_state.getStamp().toSec(), start_time.toSec()+(actions.size()*dt.toSec()));
  EXPECT_LE(final_state->position.x, 10.0);
}

//5- FINALSTASTE MULTIPLEACTION STAMPED ASYNC
TEST(MotionModels, predictFromActions_async)
{
  DeterministicNonHolonomic2D<Pose, Twist> motion_model;

  StampedData<Pose> initial_state(ros::Time(2.0), "odom");
  initial_state->orientation.w = 1.0;

  vector<StampedData<Twist> > actions;
//first action
  StampedData<Twist> cmd_vel;
  cmd_vel->linear.x = 1.0;
  cmd_vel.setStamp(ros::Time(2.0));
  actions.push_back(cmd_vel);
//second action
  cmd_vel->linear.x = -1.0;
  cmd_vel.setStamp(ros::Time(10.0));
  actions.push_back(cmd_vel);

  StampedData<Pose> final_state;
  ros::Duration simulation_duration(2.0);
  ros::Time end_simulation_time = actions.back().getStamp() + simulation_duration;

  motion_model.predictState(initial_state, actions, end_simulation_time, final_state);
  EXPECT_DOUBLE_EQ(final_state->position.x, 6.0);
  EXPECT_TRUE(final_state.getStamp()==end_simulation_time);
  EXPECT_TRUE(final_state.getFrameId()==initial_state.getFrameId());

}

//6 - SAMPLE SINGLE-ACTION NONSTAMPED SYNC
TEST(MotionModels, samplesFromSingleAction_sync)
{
  ros::Time::init();
  DeterministicNonHolonomic2D<Pose, Twist> motion_model;

  Pose initial_state;
  initial_state.orientation.w = 1.0;

  Twist cmd_vel;
  cmd_vel.linear.x = 1.0;

  vector<Pose> resulting_trajectory(10);

  ros::Rate rate(1.0);
  ros::Duration simulation_time(10);

  motion_model.sampleStates(initial_state, cmd_vel, simulation_time, resulting_trajectory);
  Pose final_state;
  motion_model.predictState(initial_state, cmd_vel, simulation_time, final_state);

  EXPECT_TRUE((resulting_trajectory.back()).position.x==final_state.position.x) << "final state: \n" << final_state
      << "\n trajectory back: \n" << resulting_trajectory.back();
  EXPECT_TRUE((resulting_trajectory.back()).position.y==final_state.position.y) << "final state: \n" << final_state
      << "\n trajectory back: \n" << resulting_trajectory.back();
  EXPECT_TRUE((resulting_trajectory.back()).orientation.z==final_state.orientation.z) << "final state: \n"
      << final_state << "\n trajectory back: \n" << resulting_trajectory.back();

  BOOST_FOREACH(Pose & sub_state, resulting_trajectory)
  {
    ROS_INFO(
        "trajectory: x %f,y %f, phi %f\n", sub_state.position.x, sub_state.position.y, tf::getYaw(sub_state.orientation));
  }
}

//7 - SAMPLE SINGLE-ACTION STAMPED SYNC
TEST(MotionModels, samples_7FromSingleActionStamped_sync)
{
  DeterministicNonHolonomic2D<Pose, Twist> motion_model;

  ros::Time initial_time(10.0);
  StampedData<Pose> initial_state(initial_time, "odom");
  initial_state->orientation.w = 1.0;

  Twist cmd_vel;
  cmd_vel.linear.x = 1.0;

  vector<StampedData<Pose> > resulting_trajectory(11);

  ros::Rate rate(1.0);
  ros::Duration simulation_time(10);

  //the first sample more one sample per second
  motion_model.sampleStates(initial_state, cmd_vel, simulation_time, resulting_trajectory);
  StampedData<Pose> final_state;
  motion_model.predictState(initial_state, cmd_vel, simulation_time, final_state);

  EXPECT_TRUE((resulting_trajectory.back())->position.x==final_state->position.x) << "final state: \n" << *final_state
      << "\n trajectory back: \n" << *(resulting_trajectory.back());
  EXPECT_TRUE((resulting_trajectory.back())->position.y==final_state->position.y) << "final state: \n" << *final_state
      << "\n trajectory back: \n" << *(resulting_trajectory.back());
  EXPECT_TRUE((resulting_trajectory.back())->orientation.z==final_state->orientation.z) << "final state: \n"
      << *final_state << "\n trajectory back: \n" << *(resulting_trajectory.back());

  ros::Time current_time(initial_time);
  double t = 0;
  BOOST_FOREACH(StampedData < Pose > &sub_state, resulting_trajectory)
  {
    EXPECT_DOUBLE_EQ(t * cmd_vel.linear.x, (float)(sub_state->position.x));
    t = t + rate.expectedCycleTime().toSec();
    EXPECT_TRUE(sub_state.getStamp().toSec()==current_time.toSec()) << sub_state.getStamp();
    current_time += rate.expectedCycleTime();

    EXPECT_EQ(sub_state.getFrameId(), initial_state.getFrameId());
  }

  ros::Duration expected_sample_interval = ros::Duration(simulation_time.toSec() / (resulting_trajectory.size() - 1));
  for (unsigned long i = 0; i < resulting_trajectory.size(); i++)
  {
    StampedData<Pose>& sub_state = resulting_trajectory[i];
    ROS_INFO(
        "trajectory:[%f] %f,%f, %f\n", sub_state.getStamp().toSec(), sub_state->position.x, sub_state->position.y, tf::getYaw(sub_state->orientation));

    if (i > 0)
    {
      ros::Duration sample_interval = sub_state.getStamp() - resulting_trajectory[i - 1].getStamp();
      ASSERT_NEAR(expected_sample_interval.toSec(), sample_interval.toSec(), expected_sample_interval.toSec()/100.0);
    }
  }
}

//8A - SAMPLE MULTIPLEACTIONS NONSTAMPED SYNC
TEST(MotionModels, samplesFromMultiple_8_A_ActionsNonStamped_sync)
{
  DeterministicNonHolonomic2D<Pose, Twist> motion_model;

  ros::Time start_time(0.0);
  Pose initial_state;
  initial_state.orientation.w = 1.0;

  vector<Twist> actions;
  Twist cmd_vel;
  cmd_vel.linear.x = 1.0;
  actions.push_back(cmd_vel);
  actions.push_back(cmd_vel);

  ros::Duration action_time(5);
  ros::Duration simulation_time = action_time * actions.size();

  Pose final_state;
  vector<Pose> trajectory(11);
  motion_model.predictState(initial_state, actions, action_time, final_state);
  motion_model.sampleStates(initial_state, actions, simulation_time, trajectory);

  //because the second action moves the robot from the straight line. then:
  EXPECT_TRUE((trajectory.back()).position.x==final_state.position.x) << "final state: \n" << final_state
      << "\n trajectory back: \n" << trajectory.back();
  EXPECT_TRUE((trajectory.back()).position.y==final_state.position.y) << "final state: \n" << final_state
      << "\n trajectory back: \n" << trajectory.back();
  EXPECT_TRUE((trajectory.back()).orientation.z==final_state.orientation.z) << "final state: \n" << final_state
      << "\n trajectory back: \n" << trajectory.back();

  BOOST_FOREACH(Pose & sub_state, trajectory)
  {
    ROS_INFO(
        "trajectory: x %f,y %f, phi %f\n", sub_state.position.x, sub_state.position.y, tf::getYaw(sub_state.orientation));
  }
}

//8B - SAMPLE MULTIPLEACTIONS NONSTAMPED SYNC
TEST(MotionModels, samplesFromMultiple_8_B_ActionsNonStamped_sync)
{
  DeterministicNonHolonomic2D<Pose, Twist> motion_model;

  ros::Time start_time(0.0);
  Pose initial_state;
  initial_state.orientation.w = 1.0;

  vector<Twist> actions;
  Twist cmd_vel;
  cmd_vel.linear.x = 1.0;
  actions.push_back(cmd_vel);
  cmd_vel.angular.z = 0.1;
  actions.push_back(cmd_vel);

  ros::Duration action_time(5);
  ros::Duration simulation_time = action_time * actions.size();

  Pose final_state;
  vector<Pose> trajectory(11);
  motion_model.predictState(initial_state, actions, action_time, final_state);
  motion_model.sampleStates(initial_state, actions, simulation_time, trajectory);

  //because the second action moves the robot from the straight line. then:
  EXPECT_TRUE((trajectory.back()).position.x==final_state.position.x) << "final state: \n" << final_state
      << "\n trajectory back: \n" << trajectory.back();
  EXPECT_TRUE((trajectory.back()).position.y==final_state.position.y) << "final state: \n" << final_state
      << "\n trajectory back: \n" << trajectory.back();
  EXPECT_TRUE((trajectory.back()).orientation.z==final_state.orientation.z) << "final state: \n" << final_state
      << "\n trajectory back: \n" << trajectory.back();

  for (unsigned long i = 0; i < trajectory.size(); i++)
  {
    Pose& sub_state = trajectory[i];
    ROS_INFO(
        "trajectory: x %f,y %f, phi %f\n", sub_state.position.x, sub_state.position.y, tf::getYaw(sub_state.orientation));
  }
}

//9 - SAMPLE MULTIPLEACTIONS STAMPED SYNC
TEST(MotionModels, samples_9_FromMultipleActionsStamped_sync)
{
  DeterministicNonHolonomic2D<Pose, Twist> motion_model;

  ros::Time start_time(10.0);
  StampedData<Pose> initial_state(start_time, "odom");
  initial_state->orientation.w = 1.0;

  vector<Twist> actions;
  Twist cmd_vel;
  cmd_vel.linear.x = 1.0;
  actions.push_back(cmd_vel);
  cmd_vel.angular.z = 0.10;
  actions.push_back(cmd_vel);

  ros::Rate rate(1.0);
  ros::Duration action_time(5.0);
  ros::Duration simulation_time = action_time * actions.size();

  StampedData<Pose> final_state;
  vector<StampedData<Pose> > trajectory(11);
  motion_model.sampleStates(initial_state, actions, simulation_time, trajectory);
  motion_model.predictState(initial_state, actions, action_time, final_state);

  //because the second action moves the robot from the straight line. then:
  EXPECT_TRUE((trajectory.back())->position.x==final_state->position.x);
  EXPECT_TRUE((trajectory.back())->position.y==final_state->position.y);
  EXPECT_TRUE((trajectory.back())->orientation.z==final_state->orientation.z);
  EXPECT_TRUE((trajectory.back()).getStamp()==final_state.getStamp());
  EXPECT_TRUE((trajectory.back()).getFrameId()==final_state.getFrameId());

  ros::Duration expected_sample_interval = ros::Duration(simulation_time.toSec() / (trajectory.size() - 1));
  for (unsigned long i = 0; i < trajectory.size(); i++)
  {
    StampedData<Pose>& sub_state = trajectory[i];
    ROS_INFO(
        "trajectory:[%f] %f,%f, %f\n", sub_state.getStamp().toSec(), sub_state->position.x, sub_state->position.y, tf::getYaw(sub_state->orientation));

    if (i > 0)
    {
      ros::Duration sample_interval = sub_state.getStamp() - trajectory[i - 1].getStamp();
      ASSERT_NEAR(expected_sample_interval.toSec(), sample_interval.toSec(), expected_sample_interval.toSec()/100.0);
    }
  }

}

//10 - SAMPLE MULTIPLEACTIONS STAMPED ASYNC
TEST(MotionModels, samples_10_FromMultipleActionsStamped_async)
{

  DeterministicNonHolonomic2D<Pose, Twist> motion_model;
  ros::Time start_time(10.0);
  StampedData<Pose> initial_state(start_time, "odom");
  initial_state->orientation.w = 1.0;
  initial_state->position.x = 30;

  vector<StampedData<Twist> > command_history;
  StampedData<Twist> cmd_vel;
  cmd_vel->linear.x = 1.0;
  cmd_vel.setStamp(start_time);
  command_history.push_back(cmd_vel);

  cmd_vel->angular.z = 0.1;
  cmd_vel.setStamp(start_time + ros::Duration(5.0));
  command_history.push_back(cmd_vel);

  ros::Rate rate(10.0);
  ros::Time end_time(start_time + ros::Duration(10));
  ros::Duration simulation_time(end_time - start_time);

  vector<StampedData<Pose> > trajectory(11);
  motion_model.sampleStates(initial_state, command_history, end_time, trajectory);

  ros::Duration expected_sample_interval = ros::Duration(simulation_time.toSec() / (trajectory.size() - 1));
  for (unsigned long i = 0; i < trajectory.size(); i++)
  {
    StampedData<Pose>& sub_state = trajectory[i];
    ROS_INFO(
        "trajectory:[%f] %f,%f, %f\n", sub_state.getStamp().toSec(), sub_state->position.x, sub_state->position.y, tf::getYaw(sub_state->orientation));

    if (i > 0)
    {
      ros::Duration sample_interval = sub_state.getStamp() - trajectory[i - 1].getStamp();
      ASSERT_NEAR(expected_sample_interval.toSec(), sample_interval.toSec(), expected_sample_interval.toSec()/100.0);
    }
  }
}

/*
 TEST(MotionModels, predictStates_mrpt_pose2d)
 {
 ros::Time::init();
 DeterministicNonHolonomic2D<mrpt::poses::CPose2D, rtcus_motion_models::Twist2D> motion_model;
 StampedData<mrpt::poses::CPose2D> initial_state(ros::Time::now(), "odom");

 rtcus_motion_models::Twist2D cmd_vel;
 cmd_vel.linear_velocity = 1.0;
 ros::Rate rate(10.0);
 ros::Duration simulation_time(1.0);

 vector<StampedData<mrpt::poses::CPose2D> > resulting_trajectory;
 motion_model.predictStates(initial_state, cmd_vel, simulation_time, rate.expectedCycleTime(), resulting_trajectory);
 ASSERT_EQ(resulting_trajectory.size(), 11);

 ros::Time current_time = initial_state.getStamp();
 double i = 0.0;
 BOOST_FOREACH(Stamped<mrpt::poses::CPose2D> & sub_state,resulting_trajectory)
 {

 ASSERT_EQ(initial_state.getFrameId(),sub_state.getFrameId()) << sub_state.getData();
 ASSERT_EQ(current_time, sub_state.getStamp());
 current_time += rate.expectedCycleTime();

 //EXPECT_FALSE(true)<<sub_state->getStamp()<<"substate"<<sub_state->getData();

 EXPECT_NEAR(i*cmd_vel.linear_velocity*rate.expectedCycleTime().toSec(), (float)(sub_state.getData().x()),
 rate.expectedCycleTime().toSec()/1000.0);
 i += 1.0;
 }
 }
 */
