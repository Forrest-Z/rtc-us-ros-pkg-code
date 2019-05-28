/*
 * point_cloud_perception_base.h
 *
 *  Created on: Jun 20, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef POINT_CLOUD_PERCEPTION_BASE_H_
#define POINT_CLOUD_PERCEPTION_BASE_H_

#include <rtcus_navigation/world_perception.h>
#include <rtcus_navigation/common.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <boost/thread.hpp>

namespace rtcus_navigation
{
namespace world_perception_ports
{

using namespace boost;

//Static obstacles
template<typename PointType>
  class PointCloudObstaclesBase : public WorldPerceptionPort<pcl::PointCloud<PointType> >
  {
  protected:
    typedef pcl::PointCloud<PointType> PointCloudType;
    //---- INPUTS -----
    //obstacle sensors
    ros::Subscriber scan_sub_;
    StampedData<PointCloudType> obstacles_data_;
    tf::TransformListener tf_listener_;
    //Params KEYs
#define SENSOR_LINK "sensor_link_frame"
#define BASE_LINK "base_link_frame"
#define DEFAULT_POINT_CLOUD_TOPIC_NAME "cloud"

    bool first_obstacles_received;
    std::string mobile_base_frame;
    std::string sensor_link_frame;

    mutex data_mutex_;
    virtual void init_subscriber()=0;

  public:

    PointCloudObstaclesBase() :
        obstacles_data_(), first_obstacles_received(false), mobile_base_frame("/base_link"), sensor_link_frame(
            "/base_laser_link")
    {

    }

    virtual void init()
    {
      this->init_subscriber();
      reset();
    }
    virtual void reset()
    {
      ros::NodeHandle& component_node_ = this->component_node_;
      if (!component_node_.getParam(SENSOR_LINK, sensor_link_frame))
        setSensorFrameName(sensor_link_frame);

      if (!component_node_.getParam(BASE_LINK, mobile_base_frame))
        setMobileBaseFrameName(mobile_base_frame);

      first_obstacles_received = false;

    }
    virtual ~PointCloudObstaclesBase()
    {
      boost::mutex::scoped_lock l(data_mutex_);
    }

    virtual bool hasValidObstaclesEstimation()
    {
      return first_obstacles_received;
    }

    virtual void setSensorFrameName(const std::string& value)
    {
      this->sensor_link_frame = value;
      this->component_node_.setParam(SENSOR_LINK, value);
    }

    virtual std::string getSensorFrameName() const
    {
      return sensor_link_frame;
    }

    virtual void setMobileBaseFrameName(const std::string& value)
    {
      this->mobile_base_frame = value;
      this->component_node_.setParam(BASE_LINK, value);
    }

    virtual std::string getMobileBaseFrameName() const
    {
      return mobile_base_frame;
    }

    //PointCloudType input
    /*! @brief
     * Thread safe method.
     * We suppose this read has been made in the sensor local frame right now
     *
     */
    virtual void pushObstacles(const PointCloudType& obstacles)
    {
      {
        mutex::scoped_lock lock(data_mutex_);
        if (ARE_DISTINCT_FRAMES(obstacles.header.frame_id, this->getSensorFrameName()))
        {
          ROS_ERROR(
              "World Perception Port. Perception data omitted. Obstacles are labeled with the frame [%s] while the expected sensor frame is [%s]", obstacles.header.frame_id.c_str(), this->getSensorFrameName().c_str());
          return;
        }
        first_obstacles_received = true;

        tf::Transform sensor_pos;
        tf::StampedTransform transform;
        try
        {
          this->tf_listener_.lookupTransform(this->sensor_link_frame, this->getMobileBaseFrameName(), ros::Time(0),
                                             transform);
          //TODO: "This has to be revised if nonstatic laser link frame is considered"
          sensor_pos = transform;

        }
        catch (tf::TransformException& exception)
        {
          ROS_ERROR("%s", exception.what());
        }

        sensor_pos.setOrigin(
            tf::Vector3(obstacles_data_->sensor_origin_[0], obstacles_data_->sensor_origin_[1],
                        obstacles_data_->sensor_origin_[2]));
        tf::Quaternion orientation(obstacles_data_->sensor_orientation_.x(), obstacles_data_->sensor_orientation_.y(),
                                   obstacles_data_->sensor_orientation_.z(), obstacles_data_->sensor_orientation_.w());

        if (orientation.x() == 0 && orientation.y() == 0 && orientation.z() == 0 && orientation.w() == 0)
        {
          ROS_ERROR(
              "World Perception Port. Perception Data ommited: While perceiving new obstacles. Invalid sensor origin in the provided point cloud. ");
          return;
        }

        sensor_pos.setRotation(tf::Quaternion(orientation));

        //TODO: This is already a copy so no aditional copies are needed for external
        rtcus_compositions::StateComposer::compose(obstacles, sensor_pos, obstacles_data_.getData());
        obstacles_data_->sensor_origin_.setZero();
        obstacles_data_->sensor_orientation_ = Eigen::Quaternionf::Identity();
        //}

        obstacles_data_.setFrameId(getMobileBaseFrameName());
        obstacles_data_.setStamp(obstacles.header.stamp);
      }
      this->onWorldUpdate(*this);
    }

    virtual shared_ptr<mutex> getWorldEstimation(ros::Time time, Stamped<PointCloudType>& obstacles)
    {
      return getLastUpdateWorld(obstacles);
    }

    virtual shared_ptr<mutex> getLastUpdateWorld(Stamped<PointCloudType>& obstacles)
    {
      data_mutex_.lock();
      obstacles = obstacles_data_;
      return shared_ptr<mutex>(&data_mutex_, mem_fn(&mutex::unlock));
    }

    //allocation by the navigation node is required since the obstacles are described in the local base frame
    //a composition to the future state estiman is required and an extra obstacle allocation is required
    virtual void allocateObstacleRepresentation(rtcus_stamp::Stamped<pcl::PointCloud<PointType> >& obstacles)
    {
      mutex::scoped_lock lock(data_mutex_);
      obstacles = rtcus_stamp::Stamped<pcl::PointCloud<PointType> >(boost::make_shared<pcl::PointCloud<PointType> >());
    }
  }
  ;

template class PointCloudObstaclesBase<pcl::PointXY> ;
template class PointCloudObstaclesBase<pcl::PointXYZ> ;
}
}

#endif /* POINT_CLOUD_PERCEPTION_BASE_H_ */
