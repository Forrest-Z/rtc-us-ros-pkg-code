#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <gtest/gtest.h>

using namespace tf;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(TFSemantic_test,TFSemantic_test)
{

  tf::TransformBroadcaster broadcaster;
  tf::TransformListener listener;

  //robot state in past
  tf::Transform robot_state_in_past;
  robot_state_in_past.setOrigin(tf::Vector3(10, 10, 10));
  robot_state_in_past.setRotation(tf::Quaternion(0, 0, 0, 1));

  broadcaster.sendTransform(StampedTransform(robot_state_in_past, ros::Time(10), "/world", "/base_link"));

  //robot state in past
  tf::Transform robot_state_now;
  robot_state_now.setOrigin(tf::Vector3(20, 20, 10));
  robot_state_now.setRotation(tf::Quaternion(0, 0, 0, 1));
  broadcaster.sendTransform(StampedTransform(robot_state_now, ros::Time(20), "/world", "/base_link"));

  tf::Transform detected_object_in_past;
  detected_object_in_past.setOrigin(tf::Vector3(1, 0, -1));
  detected_object_in_past.setRotation(tf::Quaternion(0, 0, 0, 1));
  broadcaster.sendTransform(StampedTransform(detected_object_in_past, ros::Time(10), "/base_link", "/object"));
  for (int i = 0; i < 10; i++)
  {
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  tf::StampedTransform result;
  EXPECT_THROW(listener.lookupTransform("/object", "/base_link", ros::Time(20), result), tf::ExtrapolationException);

  //
  listener.lookupTransform("/object", "/base_link", ros::Time(10), result);
  tf::Vector3 origin = result.getOrigin();
  ROS_INFO("%lf %lf %lf", origin.x(), origin.y(), origin.z());

  //asking for an intermediate position -> INTERPOLATION is made
  listener.lookupTransform("/base_link", "/world", ros::Time(15), result);
  origin = result.getOrigin();
  ROS_INFO("%lf %lf %lf", origin.x(), origin.y(), origin.z());

  listener.lookupTransform("/base_link", "/world", ros::Time(20), result);
  origin = result.getOrigin();
  ROS_INFO("%lf %lf %lf", origin.x(), origin.y(), origin.z());
}
