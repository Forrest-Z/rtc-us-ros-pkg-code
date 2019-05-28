/*
 * LocalPointCloudObstaclesPublisher.h
 *
 *  Created on: Jul 12, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef LOCALPOINTCLOUDOBSTACLESPUBLISHER_H_
#define LOCALPOINTCLOUDOBSTACLESPUBLISHER_H_

#include <rtcus_navigation/navigation_node.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/bind.hpp>

namespace rtcus_navigation_tools
{
using namespace rtcus_navigation;

template<typename NavigationNode>
  class LocalPointCloudObstaclesPublisher
  {
    USING_NAVIGATION_TYPES(NavigationNode)

    ros::Publisher p;
    ros::NodeHandle nh;
    bool publish_obstacles_point_cloud_;

    void publish_point_cloudT(const NavigationNode& sender, const TObstaclesType& resulting_obstacles)
    {

      if (publish_obstacles_point_cloud_)
      {
        pcl::PointCloud < pcl::PointXYZ > copy;
        copy.width = resulting_obstacles.width;
        copy.height = resulting_obstacles.height;
        copy.header = resulting_obstacles.header;
        for (unsigned int i = 0; i < resulting_obstacles.points.size(); i++)
        {
          pcl::PointXYZ p;
          p.x = resulting_obstacles.points[i].x;
          p.y = resulting_obstacles.points[i].y;
          p.z = 0.0;
          copy.points.push_back(p);
        }

        copy.header.stamp = ros::Time::now();
        copy.header.frame_id = sender.getBasePredictionFrame();
        p.publish(copy);
      }
    }

    void onWorldPerception(WorldPerceptionPort<TObstaclesType>& sender)
    {
      Stamped<TObstaclesType> world_update;
      shared_ptr<mutex> obstacle_mutex = sender.getLastUpdateWorld(world_update);

      ROS_INFO(
          "World Update received --> stamped at (%lf) in the frame (%s)", world_update.getStamp().toSec(), world_update.getFrameId().c_str());
    }

  public:

    LocalPointCloudObstaclesPublisher(NavigationNode& sender) :
        nh(sender.getPrivateNode().getNamespace() + "/visual_representation"), publish_obstacles_point_cloud_(false)
    {
      nh.param("publish_obstacles_point_cloud", publish_obstacles_point_cloud_, false);
      p = nh.advertise<TObstaclesType>("local_obstacles", 1);

      sender.onProcessingLocalWorldEnd.connect(
          boost::bind(&LocalPointCloudObstaclesPublisher<NavigationNode>::publish_point_cloudT, this, _1, _2));

      sender.getWorldPerceptionPort()->onWorldUpdate.connect(
          boost::bind(&LocalPointCloudObstaclesPublisher<NavigationNode>::onWorldPerception, *this, _1));

    }
  };
}
#endif /* LOCALPOINTCLOUDOBSTACLESPUBLISHER_H_ */
