/*
 * point_vector_compositions.h
 *
 *  Created on: Feb 21, 2013
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <rtcus_compositions/state_composer.h>

//TODO: remove this. and move  to a linkage when the metaprograming combinatory linckage process is done
#include <rtcus_assert/rtcus_assert.h>
#include <vector>
#include <pcl/point_types.h>
#include <rtcus_nav_msgs/Pose2D.h>
#include <Eigen/StdVector>

namespace rtcus_compositions
{

using namespace std;
//Point Vectors
//PointXY + Pose2D

template<typename VectorLikePointXY>
  void inline compose_vector_aux(const VectorLikePointXY& input_cloud, const rtcus_nav_msgs::Pose2D& transf,
                                 VectorLikePointXY& resulting_cloud)
  {
    RTCUS_ASSERT(input_cloud.size() == resulting_cloud.size());
    const double ccos = cos(transf.phi);
    const double ssin = sin(transf.phi);
    for (unsigned int i = 0; i < input_cloud.size(); i++)
    {

      const double Ax = input_cloud[i].x + transf.x;
      const double Ay = input_cloud[i].y + transf.y;

      resulting_cloud[i].x = Ax * ccos - Ay * ssin;
      resulting_cloud[i].y = Ax * ssin + Ay * ccos;
    }
  }
template<typename VectorLikePointXY>
  void inline inverse_compose_vector_aux(const VectorLikePointXY& input_cloud, const rtcus_nav_msgs::Pose2D& transf,
                                         VectorLikePointXY& resulting_cloud)
  {
    RTCUS_ASSERT(input_cloud.size() == resulting_cloud.size());
    const double ccos = cos(transf.phi);
    const double ssin = sin(transf.phi);
    for (unsigned int i = 0; i < input_cloud.size(); i++)
    {

      const double Ax = input_cloud[i].x - transf.x;
      const double Ay = input_cloud[i].y - transf.y;

      resulting_cloud[i].x = Ax * ccos + Ay * ssin;
      resulting_cloud[i].y = -Ax * ssin + Ay * ccos;
    }
  }

template<>
  void StateComposer::compose<vector<pcl::PointXY>, rtcus_nav_msgs::Pose2D>(const vector<pcl::PointXY>& input_cloud,
                                                                            const rtcus_nav_msgs::Pose2D& transf,
                                                                            vector<pcl::PointXY>& resulting_cloud)
  {
    compose_vector_aux(input_cloud, transf, resulting_cloud);
  }
//StateComposer::compose(input_cloud[i], transf, resulting_cloud[i]);

template<>
  void StateComposer::inverse_compose<vector<pcl::PointXY>, rtcus_nav_msgs::Pose2D>(
      const vector<pcl::PointXY>& input_cloud, const rtcus_nav_msgs::Pose2D& transf,
      vector<pcl::PointXY>& resulting_cloud, const std::string& new_frame_id)
  {
    inverse_compose_vector_aux(input_cloud, transf, resulting_cloud);
  }

template<>
  void StateComposer::compose<vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> >, rtcus_nav_msgs::Pose2D>(
      const vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> >& input_cloud,
      const rtcus_nav_msgs::Pose2D& transf,
      vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> >& resulting_cloud)
  {
    compose_vector_aux(input_cloud, transf, resulting_cloud);
  }
//StateComposer::compose(input_cloud[i], transf, resulting_cloud[i]);

template<>
  void StateComposer::inverse_compose<vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> >,
      rtcus_nav_msgs::Pose2D>(const vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> >& input_cloud,
                              const rtcus_nav_msgs::Pose2D& transf,
                              vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> >& resulting_cloud,
                              const std::string& new_frame_id)
  {
    inverse_compose_vector_aux(input_cloud, transf, resulting_cloud);
  }
}
