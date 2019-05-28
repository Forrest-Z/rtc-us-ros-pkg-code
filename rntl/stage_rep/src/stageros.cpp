/*
 *  stageros
 *  Copyright (c) 2008, Willow Garage, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/**

 @mainpage

 @htmlinclude manifest.html
 **/
#include <stage_node.h>
#include <dynamic_reconfigure_stageros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <sys/stat.h>
#include <boost/thread.hpp>

// since stageros is single-threaded, this is OK. revisit if that changes!
const char *
StageNode::mapName(const char *name, size_t robotID)
{
  if (positionmodels.size() > 1)
  {
    static char buf[100];
    snprintf(buf, sizeof(buf), "/robot_%u/%s", (unsigned int)robotID, name);
    return buf;
  }
  else
    return name;
}

void StageNode::ghfunc(Stg::Model* mod, StageNode* node)
{
  if (dynamic_cast<Stg::ModelRanger *>(mod))
    node->lasermodels.push_back(dynamic_cast<Stg::ModelRanger *>(mod));
  if (dynamic_cast<Stg::ModelPosition *>(mod))
    node->positionmodels.push_back(dynamic_cast<Stg::ModelPosition *>(mod));
}

void StageNode::cmdvelReceived(int idx, const boost::shared_ptr<geometry_msgs::Twist const>& msg)
{
  boost::mutex::scoped_lock lock(msg_lock);

  this->positionmodels[idx]->SetSpeed(msg->linear.x, msg->linear.y, msg->angular.z);
  //this->positionmodels[idx]->SetVelocity(command);
  this->base_last_cmd = this->sim_time;
}

StageNode::StageNode(int argc, char** argv, bool gui, const char* fname)
{
  this->sim_time.fromSec(0.0);
  this->base_last_cmd.fromSec(0.0);
  double t;
  ros::NodeHandle localn("~");
  if (!localn.getParam("base_watchdog_timeout", t))
    t = 0.2;
  this->base_watchdog_timeout.fromSec(t);

  // We'll check the existence of the world file, because libstage doesn't
  // expose its failure to open it.  Could go further with checks (e.g., is
  // it readable by this user).
  struct stat s;
  if (stat(fname, &s) != 0)
  {
    ROS_FATAL("The world file %s does not exist.", fname);
    ROS_BREAK();
  }

  // initialize libstage
  Stg::Init(&argc, &argv);

  if (gui)
    this->world = new Stg::WorldGui(800, 700, "Stage (ROS)");
  else
    this->world = new Stg::World();

  boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

  // Apparently an Update is needed before the Load to avoid crashes on
  // startup on some systems.
  // As of Stage 4.1.1, this update call causes a hang on start.
  //this->UpdateWorld();
  this->world->Load(fname);

  // We add our callback here, after the Update, so avoid our callback
  // being invoked before we're ready.
  this->world->AddUpdateCallback((Stg::world_callback_t)s_update, this);

  this->world->ForEachDescendant((Stg::model_callback_t)ghfunc, this);

  if (lasermodels.size() != positionmodels.size())
  {
    ROS_FATAL("number of position models and laser models must be equal in "
    "the world file.");
    ROS_BREAK();
  }

  size_t numRobots = positionmodels.size();
  ROS_INFO("found %u position/laser pair%s in the file", (unsigned int)numRobots, (numRobots == 1) ? "" : "s");

  this->laserMsgs = new sensor_msgs::LaserScan[numRobots];
  this->odomMsgs = new nav_msgs::Odometry[numRobots];
  this->groundTruthMsgs = new nav_msgs::Odometry[numRobots];
}

// Subscribe to models of interest.  Currently, we find and subscribe
// to the first 'laser' model and the first 'position' model.  Returns
// 0 on success (both models subscribed), -1 otherwise.
//
// Eventually, we should provide a general way to map stage models onto ROS
// topics, similar to Player .cfg files.
int StageNode::SubscribeModels()
{
  n_.setParam("/use_sim_time", true);

  for (size_t r = 0; r < this->positionmodels.size(); r++)
  {
    if (this->lasermodels[r])
    {
      this->lasermodels[r]->Subscribe();
    }
    else
    {
      ROS_ERROR("no laser");
      return (-1);
    }
    if (this->positionmodels[r])
    {
      this->positionmodels[r]->Subscribe();
    }
    else
    {
      ROS_ERROR("no position");
      return (-1);
    }

    laser_pubs_.push_back(n_.advertise<sensor_msgs::LaserScan>(mapName(BASE_SCAN, r), 10));
    odom_pubs_.push_back(n_.advertise<nav_msgs::Odometry>(mapName(ODOM, r), 10));
    ground_truth_pubs_.push_back(n_.advertise<nav_msgs::Odometry>(mapName(BASE_POSE_GROUND_TRUTH, r), 10));

    cmdvel_subs_.push_back(
        n_.subscribe<geometry_msgs::Twist>(mapName(CMD_VEL, r), 10,
                                           boost::bind(&StageNode::cmdvelReceived, this, r, _1)));
  }
  clock_pub_ = n_.advertise<rosgraph_msgs::Clock>("/clock", 10);
  return (0);
}

StageNode::~StageNode()
{
  delete[] laserMsgs;
  delete[] odomMsgs;
  delete[] groundTruthMsgs;
}

bool StageNode::UpdateWorld()
{
  return this->world->UpdateAll();
}

void StageNode::WorldCallback()
{
  boost::mutex::scoped_lock lock(msg_lock);

  this->sim_time.fromSec(world->SimTimeNow() / 1e6);
  // We're not allowed to publish clock==0, because it used as a special
  // value in parts of ROS, #4027.
  if (this->sim_time.sec == 0 && this->sim_time.nsec == 0)
  {
    ROS_DEBUG("Skipping initial simulation step, to avoid publishing clock==0");
    return;
  }

  // TODO make this only affect one robot if necessary
  if ((this->base_watchdog_timeout.toSec() > 0.0)
      && ((this->sim_time - this->base_last_cmd) >= this->base_watchdog_timeout))
  {
    for (size_t r = 0; r < this->positionmodels.size(); r++)
      this->positionmodels[r]->SetSpeed(0.0, 0.0, 0.0);
  }

  // Get latest laser data
  for (size_t r = 0; r < this->lasermodels.size(); r++)
  {
    const std::vector<Stg::ModelRanger::Sensor>& sensors = this->lasermodels[r]->GetSensors();

    if (sensors.size() > 1)
      ROS_WARN("ROS Stage currently supports rangers with 1 sensor only.");

    // for now we access only the zeroth sensor of the ranger - good
    // enough for most laser models that have a single beam origin
    const Stg::ModelRanger::Sensor& s = sensors[0];

    if (s.ranges.size())
    {
      Stg::usec_t sensor_update_interval = lasermodels[r]->GetUpdateInterval();
      ros::Time next_expected_update_time = laserMsgs[r].header.stamp
          + ros::Duration((double)sensor_update_interval / 1e6);
      if (sim_time >= next_expected_update_time)
      {
        // Translate into ROS message format and publish
        this->laserMsgs[r].angle_min = -s.fov / 2.0;
        this->laserMsgs[r].angle_max = +s.fov / 2.0;
        this->laserMsgs[r].angle_increment = s.fov / (double)(s.sample_count - 1);
        this->laserMsgs[r].range_min = s.range.min;
        this->laserMsgs[r].range_max = s.range.max;
        this->laserMsgs[r].ranges.resize(s.ranges.size());
        this->laserMsgs[r].intensities.resize(s.intensities.size());

        for (unsigned int i = 0; i < s.ranges.size(); i++)
        {
          this->laserMsgs[r].ranges[i] = s.ranges[i];
          this->laserMsgs[r].intensities[i] = (uint8_t)s.intensities[i];
        }

        this->laserMsgs[r].header.frame_id = mapName("base_laser_link", r);
        this->laserMsgs[r].header.stamp = sim_time;
        this->laser_pubs_[r].publish(this->laserMsgs[r]);
      }
    }

    // Also publish the base->base_laser_link Tx.  This could eventually move
    // into being retrieved from the param server as a static Tx.
    Stg::Pose lp = this->lasermodels[r]->GetPose();
    tf::Quaternion laserQ;
    laserQ.setRPY(0.0, 0.0, lp.a);
    tf::Transform txLaser = tf::Transform(laserQ, tf::Point(lp.x, lp.y, 0.15));
    tf.sendTransform(tf::StampedTransform(txLaser, sim_time, mapName("base_link", r), mapName("base_laser_link", r)));

    // Send the identity transform between base_footprint and base_link
    tf::Transform txIdentity(tf::createIdentityQuaternion(), tf::Point(0, 0, 0));
    tf.sendTransform(tf::StampedTransform(txIdentity, sim_time, mapName("base_footprint", r), mapName("base_link", r)));

    Stg::usec_t odometry_update_interval = this->positionmodels[r]->GetUpdateInterval();
    ros::Time next_expected_odometry_update_time = this->odomMsgs[r].header.stamp
        + ros::Duration((double)odometry_update_interval / 1e6);
    if (sim_time >= next_expected_odometry_update_time)
    {
      // Get latest odometry data
      // Translate into ROS message format and publish
      this->odomMsgs[r].pose.pose.position.x = this->positionmodels[r]->est_pose.x;
      this->odomMsgs[r].pose.pose.position.y = this->positionmodels[r]->est_pose.y;
      this->odomMsgs[r].pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->positionmodels[r]->est_pose.a);
      Stg::Velocity v = this->positionmodels[r]->GetVelocity();
      //rotate the linear velocity Rot(- this->positionmodels[r]->est_pose.y.a)

      //double ca = cos(-this->positionmodels[r]->est_pose.a);
      //double sa = sin(-this->positionmodels[r]->est_pose.a);

      this->odomMsgs[r].twist.twist.linear.x = v.x;
      this->odomMsgs[r].twist.twist.linear.y = v.y;
      //ROS_INFO(
      //    "a %lf vx %lf vy %lf-> vx %lf vy %lf", this->positionmodels[r]->est_pose.a, v.x, v.y, this->odomMsgs[r].twist.twist.linear.x, this->odomMsgs[r].twist.twist.linear.y);

      this->odomMsgs[r].twist.twist.angular.z = v.a;

      //@todo Publish stall on a separate topic when one becomes available
      //this->odomMsgs[r].stall = this->positionmodels[r]->Stall();
      //
      this->odomMsgs[r].header.frame_id = mapName("odom", r);
      this->odomMsgs[r].header.stamp = sim_time;

      this->odom_pubs_[r].publish(this->odomMsgs[r]);

      // broadcast odometry transform
      tf::Quaternion odomQ;
      tf::quaternionMsgToTF(odomMsgs[r].pose.pose.orientation, odomQ);
      tf::Transform txOdom(odomQ, tf::Point(odomMsgs[r].pose.pose.position.x, odomMsgs[r].pose.pose.position.y, 0.0));
      tf.sendTransform(tf::StampedTransform(txOdom, sim_time, mapName("odom", r), mapName("base_footprint", r)));
    }
    // Also publish the ground truth pose and velocity
    Stg::Pose gpose = this->positionmodels[r]->GetGlobalPose();
    Stg::Velocity gvel = this->positionmodels[r]->GetGlobalVelocity();
    // Note that we correct for Stage's screwed-up coord system.

    tf::Quaternion q_gpose;
    q_gpose.setRPY(0.0, 0.0, gpose.a);
    tf::Transform gt(q_gpose, tf::Point(gpose.x, gpose.y, 0.0));

    tf::Quaternion q_gvel;
    q_gvel.setRPY(0.0, 0.0, gvel.a);
    tf::Transform gv(q_gvel, tf::Point(gvel.x, gvel.y, 0.0));

    this->groundTruthMsgs[r].pose.pose.position.x = gt.getOrigin().x();
    this->groundTruthMsgs[r].pose.pose.position.y = gt.getOrigin().y();
    this->groundTruthMsgs[r].pose.pose.position.z = gt.getOrigin().z();
    this->groundTruthMsgs[r].pose.pose.orientation.x = gt.getRotation().x();
    this->groundTruthMsgs[r].pose.pose.orientation.y = gt.getRotation().y();
    this->groundTruthMsgs[r].pose.pose.orientation.z = gt.getRotation().z();
    this->groundTruthMsgs[r].pose.pose.orientation.w = gt.getRotation().w();
    this->groundTruthMsgs[r].twist.twist.linear.x = gv.getOrigin().x();
    this->groundTruthMsgs[r].twist.twist.linear.y = gv.getOrigin().y();
    //this->groundTruthMsgs[r].twist.twist.angular.z = tf::getYaw(gv.getRotation());
    //this->groundTruthMsgs[r].twist.twist.linear.x = gvel.x;
    //this->groundTruthMsgs[r].twist.twist.linear.y = gvel.y;
    this->groundTruthMsgs[r].twist.twist.angular.z = gvel.a;

    this->groundTruthMsgs[r].header.frame_id = mapName("odom", r);
    this->groundTruthMsgs[r].header.stamp = sim_time;

    this->ground_truth_pubs_[r].publish(this->groundTruthMsgs[r]);
  }

  this->clockMsg.clock = sim_time;
  this->clock_pub_.publish(this->clockMsg);
}

