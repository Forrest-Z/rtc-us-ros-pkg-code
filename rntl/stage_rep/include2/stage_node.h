/*
 * stage_node.h
 *
 *  Created on: Oct 11, 2012
 *      Author: root
 */

#ifndef STAGE_NODE_H_
#define STAGE_NODE_H_
#include <boost/shared_ptr.hpp>
#include <Stage-4.1/stage.hh>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Image.h>

#define USAGE "stageros <worldfile>"
#define ODOM "odom"
#define BASE_SCAN "base_scan"
#define BASE_POSE_GROUND_TRUTH "base_pose_ground_truth"
#define CMD_VEL "cmd_vel"
#define CAMERA "camera"

//thanks also to the patch of http://answers.ros.org/question/43238/using-stageros-and-a-simulated-camera/
class StageNode
{

public:
  // Constructor; stage itself needs argc/argv.  fname is the .world file
  // that stage should load.
  StageNode(int argc, char** argv, bool gui, const char* fname);
  virtual ~StageNode();

  // Subscribe to models of interest.  Currently, we find and subscribe
  // to the first 'laser' model and the first 'position' model.  Returns
  // 0 on success (both models subscribed), -1 otherwise.
  virtual int SubscribeModels();

  // Our callback
  void WorldCallback();

  // Do one update of the world.  May pause if the next update time
  // has not yet arrived.
  bool UpdateWorld();

  // Message callback for a MsgBaseVel message, which set velocities.
  void cmdvelReceived(int idx, const boost::shared_ptr<geometry_msgs::Twist const>& msg);

  // The main simulator object
  Stg::World* world;
protected:

  // Messages that we'll send or receive
  sensor_msgs::LaserScan *laserMsgs;
  nav_msgs::Odometry *odomMsgs;
  nav_msgs::Odometry *groundTruthMsgs;
  rosgraph_msgs::Clock clockMsg;
// roscpp-related bookkeeping
  ros::NodeHandle n_;

  // A mutex to lock access to fields that are used in message callbacks
  boost::mutex msg_lock;

  // The models that we're interested in
  std::vector<Stg::ModelRanger *> lasermodels;
  std::vector<Stg::ModelPosition *> positionmodels;
  std::vector<ros::Publisher> laser_pubs_;
  std::vector<ros::Publisher> odom_pubs_;
  std::vector<ros::Publisher> ground_truth_pubs_;
  std::vector<ros::Subscriber> cmdvel_subs_;
  ros::Publisher clock_pub_;

  // A helper function that is executed for each stage model.  We use it
  // to search for models of interest.
  static void ghfunc(Stg::Model* mod, StageNode* node);

  static bool s_update(Stg::World* world, StageNode* node)
  {
    node->WorldCallback();
    // We return false to indicate that we want to be called again (an
    // odd convention, but that's the way that Stage works).
    return false;
  }

  tf::TransformBroadcaster tf;

  // Last time that we received a velocity command
  ros::Time base_last_cmd;
  ros::Duration base_watchdog_timeout;

  // Current simulation time
  ros::Time sim_time;

// since stageros is single-threaded, this is OK. revisit if that changes!
  const char *
  mapName(const char *name, size_t robotID);

};

#endif /* STAGE_NODE_H_ */
