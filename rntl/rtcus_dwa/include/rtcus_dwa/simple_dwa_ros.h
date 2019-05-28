/*
 *  Created on: 29/11/2011
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef SIMPLE_DWA_ROS_H_
#define SIMPLE_DWA_ROS_H_

#include <ros/ros.h>
#include <rtcus_navigation/navigation_planner_base.h>
#include <dynamic_reconfigure/server.h>

//Standard Data representation used for the Navigation Architecture
#include <rtcus_nav_msgs/DynamicState2D.h>
#include <rtcus_kinodynamic_description/NonHolonomicKinoDynamicsConfig.h>
#include <rtcus_robot_shapes/polygonal_robot.h>

#include <rtcus_nav_msgs/Twist2D.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//DWA Implementation includes
#include <rtcus_dwa/simple_dwa.h>
#include <rtcus_dwa/dwa_config.h>
#include <rtcus_dwa/visual_representation/dwa_algorithm_representation.h>

//TODO: Rename this header file coherently with the classname
namespace rtcus_dwa
{
using namespace rtcus_dwa;
using namespace rtcus_navigation;
using namespace rtcus_nav_msgs;
using namespace boost;
using namespace rtcus_robot_shapes;
using namespace rtcus_kinodynamic_description;

/*
 * \brief this is a generic Local planner inspired which extends the DWA algorithm.
 * TODO: The DwaAlgorithm should be used by composition not inheritance
 * Candidate to be converted as a default
 * Navigation planner re-making generic again the data types.
 * */
template<typename GoalType>
  class DwaLocalPlannerBase : public rtcus_navigation::NavigationPlanner<DynamicState2D, pcl::PointCloud<pcl::PointXY>,
                                  Twist2D, GoalType, PolygonalRobot, NonHolonomicKinoDynamicsConfig>,
                              public Dwa2DAlgorithm<GoalType, Twist2D>

  {
  public:
    //TODO: Replace this for a Traits class
    typedef pcl::PointCloud<pcl::PointXY> PointCloudXY;DECLARE_NAVIGATION_PLANNER_TYPES_EXPLICIT(rtcus_nav_msgs::DynamicState2D, PointCloudXY,rtcus_nav_msgs::Twist2D,GoalType,PolygonalRobot,NonHolonomicKinoDynamicsConfig,rtcus_navigation::ROSTimeModel);

    virtual ~DwaLocalPlannerBase();
    virtual void init(TNavigationNode& host_node);
    virtual void reset();

    /**
     * \brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
     * \param cmd_vel Will be filled with the velocity command to be passed to the robot base
     * \return True if a valid velocity command was found, false otherwise
     */
    virtual bool computeVelocityCommands(const pcl::PointCloud<pcl::PointXY>& obstacles, const GoalType& goal,
                                         const DynamicState2D& local_state, Twist2D& resulting_action);



  protected:
    boost::mutex m_mutex_;
    dynamic_reconfigure::Server<SimpleDwaConfig> configure_server_;
    shared_ptr<TNavigationNode> host_node_;
    shared_ptr<rtcus_dwa::visual_representation::DwaVisualRepresentation<GoalType> > visual_representation_;

    /* \brief This constructor is hidden: Simple factory pattern force the use of an specialized class*/
    DwaLocalPlannerBase();
    virtual const SimpleDwaConfig& updatePlannerConfiguration(SimpleDwaConfig &config, uint32_t level);

    void onNavigationNodeConfigUpdated();
    void onRobotKinodynamicsUpdated(const rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig& kino_config);
    void onRobotShapeUpdated(const PolygonalRobot& shape);
  };

}
#endif /* SIMPLE_DWA_H_ */
