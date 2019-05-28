/*
 * test_main.cpp
 *
 *  Created on: Apr 10, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <gtest/gtest.h>
#include <rtcus_navigation/navigation_node.h>
#include <rtcus_dwa/simple_dwa_ros.h>
#include <rtcus_dwa/dwa_motion_model.h>
#include <rtcus_navigation_tools/timing_observer.h>

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_node");
  ROS_INFO("initializated ROS");
  ros::NodeHandle n;
  ROS_INFO("creating first node handle");
  ros::Time::init();
  ROS_INFO("ros starting");
  ros::start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

using namespace geometry_msgs;
using namespace nav_msgs;
using namespace boost;

using namespace rtcus_nav_msgs;
using namespace rtcus_motion_models;
using namespace rtcus_navigation;
using namespace rtcus_dwa;

void navigation_event(const NavigationNode<DynamicState2D, pcl::PointCloud<pcl::PointXY>, Twist2D, Pose2D>& nn,
                      const char* msg)
{
  ROS_INFO("Event: %s", msg);

  rtcus_stamp::StampedData<DynamicState2D> current_state_estimation = nn.getStateEstimation()->getLastStateEstimation();

  LockableData<Stamped<pcl::PointCloud<pcl::PointXY> > > world = nn.getWorldPerceptionPort()->getLastUpdateWorld();
  shared_lock<shared_mutex> lock = world.getSharedLock();

  ROS_INFO("\tNumber of obstacles %ld", world->getConstData().size());

  ROS_INFO(
      "\tCurrent State %lf, %lf", current_state_estimation.getConstData().pose.x, current_state_estimation.getConstData().pose.y);

  StampedData<Twist2D> action;
  if (nn.getStateEstimation()->getLastCommand(action))
  {
    ROS_INFO("\tLast command: %lf, %lf", action.linear, action.angular);
  }

}

void cmd_result(const NavigationNode<DynamicState2D, pcl::PointCloud<pcl::PointXY>, Twist2D, Pose2D>& nn,
                const Twist2D& result_cmd)
{
  ROS_INFO("Event: CMD RES ===> %lf %lf", result_cmd.linear, result_cmd.angular);
}

//================================ TESTS ============================================================
typedef NavigationNode<DynamicState2D, pcl::PointCloud<pcl::PointXY>, Twist2D, Pose2D> DwaNavigationNode;
typedef shared_ptr<DwaNavigationNode> DwaNavigationNodePtr;

// The fixture for testing class Foo.
class NavigationNodeTest : public ::testing::Test
{

public:

  DwaNavigationNodePtr nn;
  pcl::PointCloud<pcl::PointXY> obstacles_;

  NavigationNodeTest()
  {
    intializeNode();
  }

  virtual void SetUp()
  {
    nn->start();
  }

  virtual void TearDown()
  {
    nn->stop();
  }

  DwaNavigationNodePtr intializeNode()
  {

    if (nn == NULL)
    {
      SimpleDwaConfig dwa_config;
      dwa_config.obstacle_inflation = 1.3;
      dwa_config.v_fordwards_limit = 37.0;
      dwa_config.omega_limit = 0.9;
      dwa_config.velocity_acceleration = 1.0;
      dwa_config.velocity_brake_acceleration = 0.02;
      nn = create_default_dwa_navigation_node(dwa_config);
      simulate_perception(*nn);
    }
    return nn;

    // Objects declared here can be used by all tests in the test case for Foo.
  }
  void spinTimes(int times)
  {
    for (int i = 0; i < times; i++)
    {
      ros::spinOnce();
      ros::Duration(0.01).sleep();
    }
  }

  void spinSeconds(double seconds)
  {
    ros::Time start = ros::Time::now();
    ros::Time end = start + ros::Duration(seconds);
    while (ros::Time::now() < end)
    {
      ros::spinOnce();
      ros::Duration(0.01).sleep();
    }
  }

//===================== SIMULATION STUFF =========================

  pcl::PointCloud<pcl::PointXY>& initialize_obstacles()
  {
    if (obstacles_.size() == 0)
    {
      for (int i = 1; i < 1024; i++)
      {
        pcl::PointXY obstacle;
        obstacle.x = 10.0 * i;
        obstacle.y = i + 0.1;
        obstacles_.points.push_back(obstacle);
      }
    }
    obstacles_.sensor_orientation_.w() = 1;
    obstacles_.width = obstacles_.points.size();
    obstacles_.height = 1;

    obstacles_.header.stamp = ros::Time::now();
    obstacles_.header.frame_id = "base_laser_link";

    return obstacles_;
  }

  void simulate_perception(const NavigationNode<DynamicState2D, pcl::PointCloud<pcl::PointXY>, Twist2D, Pose2D>& nn)
  {
    pcl::PointCloud<pcl::PointXY>& obstacles = initialize_obstacles();
    nn.getWorldPerceptionPort()->pushObstacles(obstacles);
    ROS_INFO("Simulating PERCEPTION");
  }
}
;

TEST_F(NavigationNodeTest, BasicPlannerInitialization)
{

  DwaNavigationNodePtr nn = intializeNode();

  shared_ptr<rtcus_navigation::NavigationPlanner<DynamicState2D, pcl::PointCloud<pcl::PointXY>, Twist2D, Pose2D> > dwa_navigation_planner =
      nn->getNavigationPlanner();

  DynamicState2D local_state;

  Pose2D goal;
  Twist2D resulting_action;
  pcl::PointCloud<pcl::PointXY> obstacles = initialize_obstacles();
  goal.x = 100;

  dwa_navigation_planner->computeVelocityCommands(obstacles, goal, local_state, resulting_action);
  ASSERT_TRUE(resulting_action.linear!=0.0);
}

using namespace boost::signals;
TEST_F(NavigationNodeTest, BasicTimingObserver)
{
  DwaNavigationNodePtr nn = intializeNode();
  ROS_INFO("starting navigation node");
  rtcus_navigation_tools::TimeObserver<DynamicState2D, pcl::PointCloud<pcl::PointXY>, Twist2D, Pose2D> timing(nn);

  nn->init();
  spinTimes(100.0);
}

TEST_F(NavigationNodeTest, NodeEvents)
{
  DwaNavigationNodePtr nn = intializeNode();

  scoped_connection c1 = nn->onControlTaskBegin.connect(bind(&navigation_event, _1, "control task begin"));
  scoped_connection c2 = nn->onControlTaskEnd.connect(bind(&navigation_event, _1, "control task end"));
  scoped_connection c3 = nn->onProcessingLocalWorldEnd.connect(
      bind(&navigation_event, _1, "processing local world end"));

  scoped_connection c4 = nn->onStateEstimationBegin.connect(bind(&navigation_event, _1, "-- state estimation begin"));
  scoped_connection c5 = nn->onStateEstimationEnd.connect(bind(&navigation_event, _1, "-- state estimation end"));
  scoped_connection c6 = nn->onPlanningResult.connect(&cmd_result);

  ROS_INFO("Node started, it will be maintained during 10 seconds...");

  StampedData<Pose2D> simulated_goal;
  simulated_goal.x = 100;
  simulated_goal.setFrameId(nn->getReferenceFrame());
  simulated_goal.setStamp(ros::Time::now());

  nn->getGoalPort()->pushGoal(simulated_goal);

//obstacle pushing will be simulated on the controlTaskBegin event
  scoped_connection c7 = nn->onControlTaskBegin.connect(bind(&NavigationNodeTest::simulate_perception, this, _1));

  spinTimes(100.0);
}

