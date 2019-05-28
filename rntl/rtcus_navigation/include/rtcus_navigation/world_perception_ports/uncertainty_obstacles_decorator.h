/*
 * uncertainty_obstacles_decorator.h
 *
 *  Created on: Nov 9, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef UNCERTAINTY_OBSTACLES_DECORATOR_H_
#define UNCERTAINTY_OBSTACLES_DECORATOR_H_

#include <rtcus_navigation/world_perception_ports/point_cloud_perception_base.h>
#include <std_msgs/UInt64.h>
#include <rtcus_navigation/UncertaintyObstaclesPerceptionPortConfig.h>
#include <dynamic_reconfigure/server.h>

namespace rtcus_navigation
{
namespace world_perception_ports
{

using namespace boost;

template<typename PointType>
struct compare
{
  bool operator ()(const PointType& a, const PointType&b)
  {
    float ka = atan2(a.y, a.x);
    float kb = atan2(b.y, b.x);
    return ka < kb;
  }
};

//Static obstacles
template<typename PointType>
  class UncertaintyObstaclesDecorator : public WorldPerceptionPort<pcl::PointCloud<PointType> >
  {
  protected:
    shared_ptr<PointCloudObstaclesBase<PointType> > decorated_;
    boost::mutex m_mutex;
    ros::NodeHandle nh_;
    dynamic_reconfigure::Server<rtcus_navigation::UncertaintyObstaclesPerceptionPortConfig> configure_server_;
    double max_distance_;
    UncertaintyObstaclesPerceptionPortConfig config_;

    ros::Publisher artificial_points_pub_;
    ros::Publisher resulting_point_cloud_;
    ros::Publisher total_points_count_pub_;
    ros::Time last_stamp_;

    pcl::PointCloud<pcl::PointXYZ> artificial_obstacles_;
    pcl::PointCloud<pcl::PointXYZ> cutted_obstacles_;

    const UncertaintyObstaclesPerceptionPortConfig& configure_server_callback(
        UncertaintyObstaclesPerceptionPortConfig &config)
    {
      boost::mutex::scoped_lock lock(m_mutex);
      this->config_ = config;
      return this->config_;
    }

    virtual void updateParameters()
    {

      if (!nh_.getParam("max_distance", max_distance_))
        nh_.setParam("max_distance", max_distance_);

    }

  public:
    UncertaintyObstaclesDecorator(shared_ptr<PointCloudObstaclesBase<PointType> > decorated) :
        decorated_(decorated), nh_((this->component_node_.getNamespace() + std::string("/artificial_obstacles"))), configure_server_(
            nh_), max_distance_(200.0)
    {
      this->config_.close_back = true;
      this->config_.artificial_obstacle_fillup_distance = -1.0;
      this->config_.distance_cutoff = 0;
      this->artificial_points_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("new_points_cloud", 1, false);
      this->total_points_count_pub_ = nh_.advertise<std_msgs::UInt64>("count", 1, false);
      this->resulting_point_cloud_ = nh_.advertise<pcl::PointCloud<PointType> >("resulting_total_cloud", 1, false);

    }

    virtual ~UncertaintyObstaclesDecorator()
    {

    }

    virtual void reset()
    {
      this->updateParameters();
      decorated_->reset();
    }
    virtual void init()
    {
      configure_server_.setCallback(boost::bind(&UncertaintyObstaclesDecorator::configure_server_callback, this, _1));
      this->updateParameters();
      decorated_->init();
      last_stamp_ = ros::Time::now();
    }

    virtual shared_ptr<mutex> getLastUpdateWorld(Stamped<pcl::PointCloud<PointType> >& obstacles)
    {
      boost::mutex::scoped_lock lock(m_mutex);
      shared_ptr<mutex> obstacle_mutex = decorated_->getLastUpdateWorld(obstacles);
      if (this->config_.enabled)
        createArtificialObstacles(obstacles);
      return obstacle_mutex;
    }

    virtual shared_ptr<mutex> getWorldEstimation(ros::Time time, Stamped<pcl::PointCloud<PointType> >& obstacles)
    {
      boost::mutex::scoped_lock lock(m_mutex);
      shared_ptr<mutex> obstacle_mutex = decorated_->getWorldEstimation(time, obstacles);
      if (this->config_.enabled)
        createArtificialObstacles(obstacles);
      return obstacle_mutex;
    }

    virtual bool hasValidObstaclesEstimation()
    {
      return decorated_->hasValidObstaclesEstimation();
    }

    virtual void pushObstacles(const pcl::PointCloud<PointType>& obstacles)
    {
      decorated_->pushObstacles(obstacles);
    }

    virtual void setSensorFrameName(const std::string& name)
    {
      decorated_->setSensorFrameName(name);
    }

    virtual std::string getSensorFrameName() const
    {
      return decorated_->getSensorFrameName();
    }

    virtual void setMobileBaseFrameName(const std::string& name)
    {
      decorated_->setMobileBaseFrameName(name);
    }

    virtual std::string getMobileBaseFrameName() const
    {
      return decorated_->getMobileBaseFrameName();
    }

    virtual void allocateObstacleRepresentation(rtcus_stamp::Stamped<pcl::PointCloud<PointType> >& obstacles)
    {
      decorated_->allocateObstacleRepresentation(obstacles);

    }

  protected:

    void cutObstacles(Stamped<pcl::PointCloud<PointType> >& obstacles)
    {
      //TODO: implement better in this way: http://answers.ros.org/question/48583/pcl_tutorial-how-to-show-the-point-cloud/

      double distance_cuttof_squared = this->config_.distance_cutoff * this->config_.distance_cutoff;
      unsigned long previous_size = obstacles->points.size();
      //ROS_INFO("CUTTING OBSTACLES THRESHOLD %lf", this->config_.distance_cutoff);
      //THEN CUT DATA
      cutted_obstacles_.points.clear();
      if (previous_size > 0)
      {
        PointXY * current, *next1pt, *next2pt;
        double first_distance_squared = -1, second_distance_squared = -1;
        pcl::PointXYZ p;
        p.x = obstacles->points.front().x;
        p.y = obstacles->points.front().y;
        p.z = 0;
        cutted_obstacles_.points.push_back(p);
        for (unsigned long i = 0; i + 2 < previous_size; i++)
        {
          next1pt = &(obstacles->points[i + 1]);
          next2pt = &(obstacles->points[i + 2]);

          if (first_distance_squared == -1)
          {
            current = &(obstacles->points[i]);
            first_distance_squared = pow(current->x - next1pt->x, 2) + pow(current->y - next1pt->y, 2);
          }

          //typically small distance
          second_distance_squared = pow(next1pt->x - next2pt->x, 2) + pow(next1pt->y - next2pt->y, 2);

          if (first_distance_squared < distance_cuttof_squared && second_distance_squared < distance_cuttof_squared)
          {
            //THEN REMOVE POINT, IS NOT USEFUL next1pt
            i = i + 1;
            // ROS_INFO(
            //     "Cutting [%lf] %ld -> d1 %lf d2 %lf", distance_cuttof_squared, i, first_distance_squared, second_distance_squared);
            first_distance_squared = first_distance_squared + second_distance_squared;
          }
          else
          {
            first_distance_squared = second_distance_squared;
            p.x = next1pt->x;
            p.y = next1pt->y;
            p.z = 0;
            cutted_obstacles_.points.push_back(p);
          }
          second_distance_squared = -1;
        }
        //add the last point
        p.x = obstacles->points.back().x;
        p.y = obstacles->points.back().y;
        p.z = 0;
        cutted_obstacles_.points.push_back(p);
      }
      obstacles->points.resize(0);
      for (unsigned int i = 0; i < cutted_obstacles_.points.size(); i++)
      {
        pcl::PointXY p;
        p.x = cutted_obstacles_.points[i].x;
        p.y = cutted_obstacles_.points[i].y;
        obstacles->points.push_back(p);
      }
      previous_size = obstacles->points.size();

    }
    void createArtificialObstacles(Stamped<pcl::PointCloud<PointType> >& obstacles)
    {
      if (obstacles->header.stamp > last_stamp_)
      {
        //copy the header data of the new artifical obstacles
        if (this->config_.publish_obstacle_decoration_info)
        {
          this->artificial_obstacles_.clear();
          this->artificial_obstacles_.points.clear();
          this->artificial_obstacles_.resize(0);
          this->artificial_obstacles_.header.frame_id = obstacles->header.frame_id;
          this->artificial_obstacles_.header.stamp = obstacles->header.stamp;
        }

        this->last_stamp_ = obstacles->header.stamp;
        unsigned long previous_size = obstacles->points.size();
        std::sort(obstacles->points.begin(), obstacles->points.end(), compare<PointType>());

        if (this->config_.distance_cutoff > 0)
          this->cutObstacles(obstacles);

        //CREATE ARTIFICIAL OBSTACLES TROUGH DIVIDE AND CONQUER
        double squared_minimum_distance = this->config_.artificial_obstacle_fillup_distance
            * this->config_.artificial_obstacle_fillup_distance;
        double squared_maximum_distance = this->config_.artificial_obstacle_fillup_distance_max
            * this->config_.artificial_obstacle_fillup_distance_max;
        //ROS_INFO("NUMBER OF POINTS IN THE CLOUD %ld", previous_size);
        for (unsigned long i = 0; i + 1 < previous_size; i++)
        {
          PointType & current = obstacles->points[i];
          PointType & nextpt = obstacles->points[i + 1];
          //divide and conquer
          solve_distance(current, nextpt, *obstacles, squared_minimum_distance, squared_maximum_distance, obstacles);
        }

        if (config_.close_back && previous_size >= 2)
          solve_distance(obstacles->points[0], obstacles->points[previous_size - 1], *obstacles,
                         squared_minimum_distance, squared_maximum_distance, obstacles);

        obstacles->height = 1;
        obstacles->width = obstacles->points.size();

        if (this->config_.publish_obstacle_decoration_info)
        {
          std_msgs::UInt64 total_obstacle_count_;
          total_obstacle_count_.data = obstacles->points.size();
          total_points_count_pub_.publish(total_obstacle_count_);

          if (this->artificial_obstacles_.size() > 0)
          {
            this->artificial_obstacles_.height = 1;
            this->artificial_obstacles_.width = artificial_obstacles_.points.size();
            this->artificial_points_pub_.publish(this->artificial_obstacles_);
            this->cutted_obstacles_.header = obstacles->header;
            this->resulting_point_cloud_.publish(this->cutted_obstacles_);
          }
        }
      }
    }

    void solve_distance(pcl::PointXY& a, pcl::PointXY&b, pcl::PointCloud<pcl::PointXY>& point_cloud,
                        double squared_minimum_distance, double squared_maximum_distance,
                        Stamped<pcl::PointCloud<PointType> >& obstacles)
    {
      //ROS_INFO("computing distance from point %lf %lf to %lf %lf", a.x, a.y, b.x, b.y);
      if (fabs(a.x - b.x) > max_distance_ || fabs(a.y - b.y) > max_distance_)
        return;

      double squared_distance = pow(a.x - b.x, 2) + pow(a.y - b.y, 2);
      //ROS_INFO("distance from (%f %f) to (%f %f)-> %lf", a.x, a.y, b.y, b.y, squared_distance);
      if (squared_distance > squared_minimum_distance && squared_distance < squared_maximum_distance)
      {
        pcl::PointXY artificial_point;
        artificial_point.x = (a.x + b.x) / 2.0;
        artificial_point.y = (a.y + b.y) / 2.0;

        //ROS_INFO("creating point %lf %lf", artificial_point.x, artificial_point.y);
        if (this->config_.publish_obstacle_decoration_info)
        {
          pcl::PointXYZ added_point;
          rtcus_conversions::Conversions::convert(artificial_point, added_point);
          //ROS_INFO("artificial->capacity: %ld, artificial->size(): %ld", artificial_obstacles_.points.capacity(),
          //         artificial_obstacles_.points.size());
          //ROS_INFO("obstacles->capacity: %ld, obstacles->size(): %ld", obstacles->points.capacity(),
          //        obstacles->points.size());
          this->artificial_obstacles_.points.push_back(added_point);
          obstacles->points.push_back(artificial_point);

        }
        solve_distance(a, artificial_point, point_cloud, squared_minimum_distance, squared_maximum_distance, obstacles);
        solve_distance(artificial_point, b, point_cloud, squared_minimum_distance, squared_maximum_distance, obstacles);
      }
    }
  };
}
}

#endif /* UNCERTAINTY_OBSTACLES_DECORATOR_H_ */
