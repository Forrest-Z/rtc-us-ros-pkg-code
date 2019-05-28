/*
 * stamp_factory.cpp
 *
 *  Created on: Apr 10, 2012
 *      Author: geus
 */

#include <rtcus_stamp/stamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <gtest/gtest.h>
#include <stdio.h>

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


using namespace rtcus_stamp;
using namespace geometry_msgs;

TEST(Stamp, BasicExamples)
{

  StampedData<geometry_msgs::Pose> stamped_pose(ros::Time(0),"root");
  printf("%f",(float)stamped_pose.getData().position.x);
  ASSERT_TRUE(true);
}

TEST(Stamp, StampedMsg)
{

  PoseStamped p;
  p.header.stamp=ros::Time(11);
  StampedMsg<geometry_msgs::PoseStamped> stamped_pose(p);
  printf("seconds -> %f",(float)(stamped_pose.getStamp().toSec()));
  ASSERT_TRUE(true);

}
