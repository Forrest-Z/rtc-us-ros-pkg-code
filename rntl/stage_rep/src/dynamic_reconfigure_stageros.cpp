/*
 * dynamic_reconfigure_stageros.cpp
 *
 *  Created on: Sep 13, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 */
#include <stage_node.h>
#include <dynamic_reconfigure_stageros.h>
#include <stage_rep/RobotStatus.h>
#include <rtcus_kinodynamic_description/NonHolonomicKinoDynamicsConfig.h>
#include <boost/filesystem.hpp>
#include <rtcus_robot_shapes/polygonal_robot.h>
#include <algorithm>
#include <boost/foreach.hpp>
#include <std_msgs/Float32.h>

#include <cv_bridge/cv_bridge.h>

#include <worldfile.hh>
#define CMD_VEL_STAMPED "cmd_vel_stamped"

DynamicReconfigureStageros::DynamicReconfigureStageros(int argc, char** argv, bool gui, const char* fname) :
    StageNode(argc, argv, gui, fname), discard_old_commands_(false), configure_server_(), robot_configure_server_(
        ros::NodeHandle("~/robot_properties"))
{
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  /*GLOBAL CONFIGURE SERVERS*/
  ROS_INFO("Setting up dynamic reconfigure servers...");
  dynamic_reconfigure::Server<stage_rep::StageSimulatorConfig>::CallbackType f;
  f = boost::bind(&DynamicReconfigureStageros::configure_server_callback, this, _1, _2);
  configure_server_.setCallback(f);

  dynamic_reconfigure::Server<stage_rep::RobotConfig>::CallbackType f2;
  f2 = boost::bind(&DynamicReconfigureStageros::configure_robot_callback, this, _1, _2);
  robot_configure_server_.setCallback(f2);

  this->kinodynamic_config_server_ = boost::shared_ptr<dynamic_reconfigure::Server<NonHolonomicKinoDynamicsConfig> >(
      new dynamic_reconfigure::Server<NonHolonomicKinoDynamicsConfig>(ros::NodeHandle("~kinodynamic_description")));

  kinodynamic_config_server_->setCallback(
      boost::bind(&DynamicReconfigureStageros::configure_kinodynamics_server_callback, this, _1, _2));

  //--------------------------------------------------------
  this->robot_shape_dynamic_reconfigure_server_ = boost::shared_ptr<
      dynamic_reconfigure::Server<rtcus_robot_shapes::PolygonalRobot> >(
      new dynamic_reconfigure::Server<rtcus_robot_shapes::PolygonalRobot>(ros::NodeHandle("~robot_shape")));

  this->robot_shape_dynamic_reconfigure_server_->setCallback(
      boost::bind(&DynamicReconfigureStageros::configure_robot_shape_server_callback, this, _1, _2));

  ROS_INFO("CREATING PUBLISHERS AND SUBSCRIBERS FOR ROBOTS");

  //--------------------------------------------------------
  this->action_delay_pub_ = nh.advertise<std_msgs::Float32>("action_delays", 10);

  const char* robot_name;
  /*TOPICS AND CONFIGURE SERVER FOR EACH ROBOT*/
  for (unsigned int i = 0; i < this->positionmodels.size(); i++)
  {
    robot_name = this->mapName("position", i);
    this->position_models_topics_.push_back(
        nh.subscribe<geometry_msgs::Pose>(std::string(robot_name), 100,
                                          boost::bind(&DynamicReconfigureStageros::on_position_callback, this, i, _1)));
  }

  if (!private_nh.getParam("discard_old_commands", discard_old_commands_))
    private_nh.setParam("discard_old_commands", discard_old_commands_);

  this->stalled_pub_ = nh.advertise<stage_rep::RobotStatus>("robot_status_truth", 10);

  if (cameramodels.size() > 0 && cameramodels.size() != positionmodels.size())
  {
    ROS_FATAL("number of position models and camera models must be equal in the world file.");
    ROS_BREAK();
  }

  size_t numRobots = positionmodels.size();
  this->imageMsgs = new sensor_msgs::Image[numRobots];

  //Stg::world_callback_t callback = (Stg::world_callback_t)boost::bind(
  //    &DynamicReconfigureStageros::futher_world_callback, this, _1, _2);
  //this->world->AddUpdateCallback(callback, this);
  this->world->AddUpdateCallback((Stg::world_callback_t)&DynamicReconfigureStageros::futher_s_update, this);
  this->world->ForEachDescendant((Stg::model_callback_t)&DynamicReconfigureStageros::extended_ghfunc, this);

  ROS_INFO("Full initialization done.");

}

DynamicReconfigureStageros::~DynamicReconfigureStageros()
{
  delete[] imageMsgs;
}

int DynamicReconfigureStageros::SubscribeModels()
{

  StageNode::SubscribeModels();
  for (size_t r = 0; r < this->positionmodels.size(); r++)
  {
    if (this->cameramodels[r])
    {
      this->cameramodels[r]->Subscribe();
    }
    else
    {
      ROS_WARN("no camera for robot %ld", r);
      return (-1);
    }

    image_pubs_.push_back(n_.advertise<sensor_msgs::Image>(mapName(CAMERA, r), 10));

    this->stamped_cmd_vel_topics_.push_back(
        n_.subscribe<rtcus_nav_msgs::StampedTwist2D>(
            mapName(CMD_VEL_STAMPED, r), 10,
            boost::bind(&DynamicReconfigureStageros::stampedCmdvelReceived, this, r, _1)));
  }
  return 0;
}

inline bool less_than_key(const rtcus_nav_msgs::StampedTwist2D& a, const rtcus_nav_msgs::StampedTwist2D& b)
{
  return (a.header.stamp.toSec() < b.header.stamp.toSec());
}

// Message callback for a MsgBaseVel message, which set velocities.
void DynamicReconfigureStageros::stampedCmdvelReceived(
    int idx, const boost::shared_ptr<rtcus_nav_msgs::StampedTwist2D const>& msg)
{
  boost::mutex::scoped_lock(msg_lock);
  std_msgs::Float32 delay;
  double sec_diff = msg->header.stamp.toSec() - ros::Time::now().toSec();
  delay.data = sec_diff;
  this->action_delay_pub_.publish(delay);
  if (sec_diff > 0)
  {
    //ROS_WARN(
    //    "Action stored. Received at (%lf). wanted to be applied at (%lf)", ros::Time::now().toSec(), msg->header.stamp.toSec());
    this->stamped_actions_buffer_.push_back(*msg);
    this->stamped_actions_buffer_.sort(less_than_key);
  }
  else if (!discard_old_commands_ || msg->header.stamp.toSec() == ros::Time::now().toSec())
  {
    this->positionmodels[idx]->SetSpeed(msg->twist.linear, msg->twist.lateral, msg->twist.angular);
    this->base_last_cmd = this->sim_time;
  }
  else
  {
    ROS_WARN(
        "Action Received at (%lf) discarded [discard_old_commands=true]. wanted to be applied at (%lf) [%lf secs late]", ros::Time::now().toSec(), msg->header.stamp.toSec(), ros::Time::now().toSec()- msg->header.stamp.toSec());
  }
}

void DynamicReconfigureStageros::extended_ghfunc(Stg::Model* mod, DynamicReconfigureStageros* node)
{
  if (dynamic_cast<Stg::ModelCamera *>(mod))
    node->cameramodels.push_back(dynamic_cast<Stg::ModelCamera *>(mod));
}

int DynamicReconfigureStageros::futher_s_update(Stg::World* world, DynamicReconfigureStageros* node)
{
  node->futher_world_callback(node->world, node);
  return 0;
}

/*THIS IS A THREAD SAFE METHOD*/
int DynamicReconfigureStageros::futher_world_callback(Stg::World* world, DynamicReconfigureStageros* thisnclass)
{
  boost::mutex::scoped_lock(msg_lock);

  this->sim_time.fromSec(world->SimTimeNow() / 1e6);
// We're not allowed to publish clock==0, because it used as a special
// value in parts of ROS, #4027.
  if (this->sim_time.sec == 0 && this->sim_time.nsec == 0)
  {
    ROS_DEBUG("Skipping initial simulation step, to avoid publishing clock==0");
    return 0;
  }

  //APPLY ACTIONS
  while (this->stamped_actions_buffer_.size() > 0)
  {
    const rtcus_nav_msgs::StampedTwist2D& stored_action = this->stamped_actions_buffer_.front();
    if (stored_action.header.stamp.toSec() <= ros::Time::now().toSec())
    {
      this->positionmodels[0]->SetSpeed(stored_action.twist.linear, stored_action.twist.lateral,
                                        stored_action.twist.angular);
      this->base_last_cmd = ros::Time::now();
      //ROS_INFO("Appling old action stamped at %lf", stored_action.header.stamp.toSec());
      this->stamped_actions_buffer_.pop_front();

    }
    else
      break;

  }
  /*
   int i = 0;
   BOOST_FOREACH(rtcus_nav_msgs::StampedTwist2D & cmd, this->stamped_actions_buffer_)
   {
   ROS_INFO("Action buffer[%d] at [%lf]", i++, cmd.header.stamp.toSec());
   }
   */
  for (size_t r = 0; r < this->positionmodels.size(); r++)
  {
    /*send the status of all position models*/
    Stg::ModelPosition* robot = this->positionmodels[r];
    if (robot->Stalled())
    {
      stage_rep::RobotStatus msg;
      msg.status = stage_rep::RobotStatus::STALLED;
      msg.stamp = this->clockMsg.clock;
      msg.robot_index = r;

      this->stalled_pub_.publish(msg);
    }

    Stg::usec_t camera_update_interval = this->cameramodels[r]->GetUpdateInterval();
    //watch the last msg stamp to see if we send again another camera msg
    ros::Time next_expected_camera_update_time = this->imageMsgs[r].header.stamp
        + ros::Duration((double)camera_update_interval / 1e6);
    if (sim_time >= next_expected_camera_update_time)
      if (this->cameramodels[r]->FrameColor())
      {
        this->imageMsgs[r].height = this->cameramodels[r]->getHeight();
        this->imageMsgs[r].width = this->cameramodels[r]->getWidth();
        this->imageMsgs[r].encoding = "rgba8";
        //this->imageMsgs[r].is_bigendian="";
        this->imageMsgs[r].step = this->imageMsgs[r].width * 4;
        this->imageMsgs[r].data.resize(this->imageMsgs[r].width * this->imageMsgs[r].height * 4);
        memcpy(&(this->imageMsgs[r].data[0]), this->cameramodels[r]->FrameColor(),
               this->imageMsgs[r].width * this->imageMsgs[r].height * 4);

        if (res.cols != this->imageMsgs[r].width || res.rows != this->imageMsgs[r].height)
          res.create(this->imageMsgs[r].height, this->imageMsgs[r].width, CV_8UC3);

        cv_bridge::CvImagePtr res2 = cv_bridge::toCvCopy(this->imageMsgs[r], "rgb8");
        cv::flip(res2->image, res, 0);
        std_msgs::Header hd;
        hd.frame_id = mapName("image", r);
        hd.stamp = sim_time;
        cv_bridge::CvImage final_msg(hd, "rgb8", res);

        this->image_pubs_[r].publish(final_msg.toImageMsg());
      }

  }

  if (shape_request.getPoints().size() > 0)
  {
    Stg::ModelPosition* robot = this->positionmodels[0];

    std::vector<Stg::point_t> polygon(shape_request.getPoints().size());
    for (unsigned int i = 0; i < shape_request.getPoints().size(); i++)
    {
      const pcl::PointXY& p = shape_request.getPoints()[i];
      polygon[i].x = p.x;
      polygon[i].y = p.y;
    }
    robot->ClearBlocks();
    robot->AddBlockPolygon(polygon, 1.0);
    this->shape_request.clear();
    ROS_INFO("ROBOT SHAPE CHANGE DONE.");
  }

  return 0;
}

void DynamicReconfigureStageros::configure_server_callback(stage_rep::StageSimulatorConfig &config, uint32_t level)
{
  boost::mutex::scoped_lock(msg_lock);
  config.map_filepath = change_map(config.map_filepath.c_str());
}

void DynamicReconfigureStageros::configure_robot_callback(stage_rep::RobotConfig &config, uint32_t level)
{
  boost::mutex::scoped_lock(msg_lock);

  //for (unsigned int i = 0; i < request.models_configurations.size(); i++)
  //{

  Stg::ModelRanger::Sensor& s = this->lasermodels[0]->GetSensorsMutable()[0];
  s.sample_count = config.sample_count;

  Stg::ModelPosition* robot = this->positionmodels[0];
  robot->actuation_noise_parameters[0]= config.actuation_noise_alpha_0;
  robot->actuation_noise_parameters[1]= config.actuation_noise_alpha_1;
  robot->actuation_noise_parameters[2]= config.actuation_noise_alpha_2;
  robot->actuation_noise_parameters[3]= config.actuation_noise_alpha_3;
  robot->actuation_noise_parameters[4]= config.actuation_noise_alpha_4;
  robot->actuation_noise_parameters[5]= config.actuation_noise_alpha_5;

  /*if (current_msg_config.laser_samples != -1)
   laser_config.sample_count = current_msg_config.laser_samples; ///< Number of range samples

   if (current_msg_config.laser_resolution != -1)
   laser_config.resolution = current_msg_config.laser_resolution; ///< interpolation

   if (current_msg_config.laser_range_min != -1)
   laser_config.range_bounds.min = current_msg_config.laser_range_min; ///< min and max ranges

   if (current_msg_config.laser_range_max != -1)
   laser_config.range_bounds.max = current_msg_config.laser_range_max;

   if (current_msg_config.laser_fov != -1.0)
   laser_config.fov = current_msg_config.laser_fov; ///< Field of view, centered about the pose angle
   */
  //stg_usec_t interval;
}

void DynamicReconfigureStageros::configure_kinodynamics_server_callback(
    rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig &config, uint32_t level)
{
  boost::mutex::scoped_lock(msg_lock);
  ROS_INFO("RECONFIGURE -STAGE ROBOT KINODINAMICS CHANGED");
  //TODO: easily generalizable for many robots
  Stg::ModelPosition* robot = this->positionmodels[0];
  robot->acceleration_bounds[0].max = config.linear_acceleration_limit;
  robot->acceleration_bounds[0].min = -config.linear_brake_limit;

  robot->acceleration_bounds[1].max = 0;
  robot->acceleration_bounds[1].min = 0;

  robot->acceleration_bounds[3].max = config.angular_acceleration_limit;
  robot->acceleration_bounds[3].min = -config.angular_acceleration_limit;

  robot->velocity_bounds[0].max = config.linear_forward_speed_limit;
  robot->velocity_bounds[0].min = config.linear_backwards_speed_limit;

  robot->velocity_bounds[1].max = 0;
  robot->velocity_bounds[1].min = 0;

  robot->velocity_bounds[3].max = config.angular_speed_limit;
  robot->velocity_bounds[3].min = -config.angular_speed_limit;

  ROS_INFO("ROBOT KINODYNAMICS CHANGE DONE.");
}

void DynamicReconfigureStageros::configure_robot_shape_server_callback(rtcus_robot_shapes::PolygonalRobot &config,
                                                                       uint32_t level)
{
  boost::mutex::scoped_lock(msg_lock);
  ROS_INFO("RECONFIGURE -STAGE ROBOT ROBOT SHAPE CHANGE REQUEST");
  //TODO: easily generalizable for many robots
  this->shape_request = config;
}

void DynamicReconfigureStageros::on_position_callback(int robot_index, const geometry_msgs::Pose::ConstPtr& msg)
{
  boost::mutex::scoped_lock(msg_lock);
  Stg::ModelPosition* robot = this->positionmodels[robot_index];
  Stg::Pose pose;
  pose.x = msg->position.x;
  pose.y = msg->position.y;
  tf::Quaternion rot;
  tf::quaternionMsgToTF(msg->orientation, rot);
  pose.a = tf::getYaw(rot);
  robot->SetPose(pose);
  robot->SetSpeed(0, 0, 0);
  robot->SetStall(false);
  ROS_INFO_STREAM("new robot pose received: " << *msg);
}

std::string DynamicReconfigureStageros::change_map(const char* map_file)
{
  ROS_INFO("STAGE RECONFIGURE - WORLD MAP CHANGE REQUEST");
  //ignore default inputs
  if (!map_file || strcmp("", map_file) == 0)
  {
    ROS_WARN("INVALID MAP NAME: %s (Ignoring request)", map_file);
    return "";
  }

  Stg::Worldfile* wf = world->GetWorldFile();
  int map_entity = wf->LookupEntity("model");
  ROS_INFO("changing simulator stage map... looking in the directory: %s", wf->filename.c_str());
  char* workaround_const = strdup(wf->filename.c_str());
  boost::filesystem::path full_path = boost::filesystem::path(
      std::string(dirname(workaround_const)) + "/" + std::string(map_file));
  ROS_INFO("Opening the file: %s", full_path.c_str());
  bool file_exist = boost::filesystem::exists(full_path) && !boost::filesystem::is_directory(full_path);

  std::string bitmapfile;
  if (!file_exist)
  {
    ROS_ERROR("The world file %s does not exist.", full_path.string().c_str());
    return "";
  }
  else //OK APPLY CHANGE
  {
    //Stg::Model* ground = this->stage_node->world->GetGround();
    Stg::Model* ground = world->GetModel("willow");
    if (!ground)
    {
      ROS_ERROR(
          "Imposible to reload the new map '%s' dynamically. The current map entity must be tagged with the name 'willow' for identifying it.", map_file);
    }
    else
    {
      ground->ClearBlocks();
      world->RemoveModel(ground);
      if (wf->PropertyExists(map_entity, "bitmap"))
      {
        bitmapfile = wf->ReadString(map_entity, "bitmap", "");
        printf("exist property bitmap: %s\n", bitmapfile.c_str());
        //Stg::CProperty * prop = wf->GetProperty(map_entity, "bitmap");
        //prop->values.clear();
        //prop->used = false;
        bitmapfile = wf->ReadString(map_entity, "bitmap", "");
        printf("exist property bitmap: %s\n", bitmapfile.c_str());
        //wf->SetPropertyValue("bitmap", 0, map_file)
        wf->WriteString(map_entity, "bitmap", map_file);

        //printf("exist property bitmap: %s\n", bitmapfile.c_str());
      }
      //stage_node->world->UnLoad();
      world->LoadModel(wf, map_entity);
      world->Redraw();
      ground = world->GetModel("willow");
      ground->Redraw();
      ground->NeedRedraw();
      ground->SetPose(ground->GetPose());
      ROS_INFO("Map change done.");
    }
    bitmapfile = wf->ReadString(map_entity, "bitmap", "");
    return bitmapfile;
  }
}

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    puts(USAGE);
    exit(-1);
  }

  ros::init(argc, argv, "stageros");

  bool gui = true;
  for (int i = 0; i < (argc - 1); i++)
  {
    if (!strcmp(argv[i], "-g"))
      gui = false;
  }

  DynamicReconfigureStageros sn(argc - 1, argv, gui, argv[argc - 1]);

  if (sn.SubscribeModels() != 0)
    exit(-1);

  boost::thread t = boost::thread(boost::bind(&ros::spin));

  // New in Stage 4.1.1: must Start() the world.
  sn.world->Start();
  // TODO: get rid of this fixed-duration sleep, using some Stage builtin
  // PauseUntilNextUpdate() functionality.
  ros::WallRate r(10.0);
  while (ros::ok() && !sn.world->TestQuit())
  {
    if (gui)
      Fl::wait(r.expectedCycleTime().toSec());
    else
    {
      sn.UpdateWorld();
      r.sleep();
    }
  }
  t.join();

  exit(0);
}

