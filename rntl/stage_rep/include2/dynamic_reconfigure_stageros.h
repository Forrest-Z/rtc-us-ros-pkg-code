/*
 * dynamic_reconfigure_stageros.h
 *
 *  Created on: Oct 11, 2012
 *      Author: root
 */

#ifndef DYNAMIC_RECONFIGURE_STAGEROS_H_
#define DYNAMIC_RECONFIGURE_STAGEROS_H_
#include <stage_node.h>
#include <stage_rep/StageSimulatorConfig.h>
#include <stage_rep/RobotConfig.h>
#include <dynamic_reconfigure/server.h>
#include <rtcus_kinodynamic_description/NonHolonomicKinoDynamicsConfig.h>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <rtcus_robot_shapes/polygonal_robot.h>
#include <rtcus_nav_msgs/StampedTwist2D.h>
#include <opencv2/opencv.hpp>

using namespace rtcus_kinodynamic_description;
using namespace rtcus_robot_shapes;

class DynamicReconfigureStageros : public StageNode
{
public:
  DynamicReconfigureStageros(int argc, char** argv, bool gui, const char* fname);
  virtual ~DynamicReconfigureStageros();
  virtual int SubscribeModels();

protected:
  dynamic_reconfigure::Server<stage_rep::StageSimulatorConfig> configure_server_;
  dynamic_reconfigure::Server<stage_rep::RobotConfig> robot_configure_server_;
  boost::shared_ptr<dynamic_reconfigure::Server<NonHolonomicKinoDynamicsConfig> > kinodynamic_config_server_;
  boost::shared_ptr<dynamic_reconfigure::Server<PolygonalRobot> > robot_shape_dynamic_reconfigure_server_;

  /*TO ACCEPT STATIC POSITIONING OR POSE CONTROL*/
  std::list<ros::Subscriber> position_models_topics_;

  /*TO ACCEPT STAMPED ACTIONS FOR TO BE STORED AND APLIED LATER*/
  std::list<rtcus_nav_msgs::StampedTwist2D> stamped_actions_buffer_;
  std::list<ros::Subscriber> stamped_cmd_vel_topics_;
  ros::Publisher action_delay_pub_;

  /*FOR CAMERA FUNCTIONALITY*/
  std::vector<ros::Publisher> image_pubs_;
  sensor_msgs::Image *imageMsgs;
  cv::Mat res;
  std::vector<Stg::ModelCamera *> cameramodels;

  ros::Publisher stalled_pub_;
  bool discard_old_commands_;

  static int futher_s_update(Stg::World* world, DynamicReconfigureStageros* node);
  static void extended_ghfunc(Stg::Model* mod, DynamicReconfigureStageros* node);

  void stampedCmdvelReceived(int idx, const boost::shared_ptr<rtcus_nav_msgs::StampedTwist2D const>& msg);

  int futher_world_callback(Stg::World* world, DynamicReconfigureStageros* thisnclass);
  void configure_server_callback(stage_rep::StageSimulatorConfig &config, uint32_t level);
  void configure_robot_callback(stage_rep::RobotConfig &config, uint32_t level);

  void configure_kinodynamics_server_callback(NonHolonomicKinoDynamicsConfig &config, uint32_t level);

  PolygonalRobot shape_request;
  void configure_robot_shape_server_callback(rtcus_robot_shapes::PolygonalRobot &config, uint32_t level);

  void on_position_callback(int robot_index, const geometry_msgs::Pose::ConstPtr& msg);
  std::string change_map(const char* map_file);
};

#endif /* DYNAMIC_RECONFIGURE_STAGEROS_H_ */
