/*
 * create_default_dwa_node.cpp
 *
 *  Created on: Jul 10, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */
#include <rtcus_trajectory_rollout_local_planner/trajectory_rollout_ros_local_planner.h>

#include <rtcus_navigation/navigation_node.h>
#include <rtcus_navigation/impl/navigation_node.h>

//rtcus_navigation existing components
#include <rtcus_navigation/world_perception_ports/laser_scan.h>
#include <rtcus_navigation/world_perception_ports/uncertainty_obstacles_decorator.h>
#include <rtcus_navigation/action_ports/adaptable_action_port.h>
#include <rtcus_navigation/goal_ports/adaptable_conversion_goal_port.h>
#include <rtcus_navigation/state_estimation/non_time_correction_state_estimation.h>
#include <rtcus_navigation/state_ports/adaptable_conversion_state_port.h>
#include <rtcus_navigation/navigation_node_factory.h>
#include <rtcus_navigation/world_perception_ports/cost_map.h>
#include <rtcus_navigation/collision_checkers/no_collision_checking.h>
#include <rtcus_navigation/goal_reach_detection/default_goal_reach.h>
#include <rtcus_navigation/shape_models/configurable_static_robot_shape.h>
#include <rtcus_navigation/kinodynamic_models/configurable_static_robot_kinodynamics.h>
#include <rtcus_kinodynamic_description/NonHolonomicKinoDynamicsConfig.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

using namespace geometry_msgs;
using namespace boost;
using namespace rtcus_navigation;
using namespace rtcus_nav_msgs;
using namespace rtcus_motion_models;
using namespace rtcus_navigation;
using namespace rtcus_navigation::state_estimation;
using namespace rtcus_navigation::world_perception_ports;
using namespace rtcus_navigation::state_ports;
using namespace rtcus_navigation::action_ports;
using namespace rtcus_navigation::goal_ports;
using namespace rtcus_navigation::goal_reach_detection;
using namespace rtcus_navigation::kinodynamic_models;
using namespace rtcus_navigation::shape_models;
using namespace rtcus_kinodynamic_description;

namespace rtcus_navigation
{
template<>
  NavigationNodeFactory<TrajectoryRolloutAlgorithmROS>::TNavigationNodePtr NavigationNodeFactory<
      TrajectoryRolloutAlgorithmROS>::create_navigation_node()
  {

    TNavigationNodePtr nn = make_shared<TNavigationNode>();

    nn->setActionPort(make_shared<ROSAdaptableActionPort<Twist2D, Twist> >());
    nn->setStatePort(make_shared<AdaptableConversionStatePort<DynamicState2D, Odometry> >());
    nn->setWorldPerceptionPort(make_shared<CostMapPerceptionPort>());
    nn->setGoalPort(make_shared<AdaptableConversionGoalPort<pcl::PointXY, PoseStamped> >());

    shared_ptr<TNavigationPlanner> planner = make_shared<TrajectoryRolloutAlgorithmROS>();
    nn->setNavigationPlanner(planner);

    nn->setReachedGoalPredicate(make_shared<DefaultGoalReachPredicate>());

    nn->setStateEstimation(make_shared<state_estimation::NonTimeCorrectionStateEstimation<DynamicState2D, Twist2D> >());

    //collision checking is integrated in the local planner and the costmap model
    nn->setCollisionChecker(
        make_shared<
            rtcus_navigation::collision_checkers::NoCollisionChecking<costmap_2d::Costmap2DROS,
                rtcus_robot_shapes::PolygonalRobot> >());

    nn->setRobotShapeModel(
        make_shared<ConfigurableRobotShapeModel<rtcus_robot_shapes::PolygonalRobot> >(
            dynamic_pointer_cast<RobotShapeModel<rtcus_robot_shapes::PolygonalRobot> >(
                make_shared<DefaultStaticRobotShape<rtcus_robot_shapes::PolygonalRobot> >())));

    nn->setKinodynamicModel(make_shared<ConfigurableKinodynamicModel<NonHolonomicKinoDynamicsConfig> >());

//FOR TESTING INIT WITH AN ARTIFICIAL GOAL
    StampedData<pcl::PointXY> goal;
    goal.getData().x = 15;
    goal.getData().y = 0;
    goal.setStamp(ros::Time(0));
    goal.setFrameId(nn->getReferenceFrame());
    nn->getGoalPort()->pushGoal(goal);

    nn->init(true);
    ROS_INFO("Trajectory Rollout NavigationNode");

    return nn;
  }
}

namespace rtcus_compositions
{
template<>
  void StateComposer::inverse_compose(const costmap_2d::Costmap2DROS& target,
                                      const rtcus_nav_msgs::DynamicState2D& local_frame_state,
                                      costmap_2d::Costmap2DROS& dst, const std::string& new_frame_name)
  {
    RTCUS_ASSERT_MSG(false, "Costmap does not allow inverse composition");
  }

// FAKE DYNAMIC STATE COMPOSER SINCE IT WILL BE USED FOR THE KINEMATIC MOTION MODEL AND ONLY POSE VARIABLES
// ARE IMPORTANT
template<>
  void StateComposer::compose<DynamicState2D, DynamicState2D>(const DynamicState2D& state,
                                                              const DynamicState2D& transform, DynamicState2D& dst)
  {
    StateComposer::compose(state.pose, transform.pose, dst.pose);
    //Supposing the local_reference_frame is static (not moving)
    dst.twist = state.twist;
  }

//FAKE DYNAMIC STATE COMPOSER
template<>
  void StateComposer::inverse_compose<DynamicState2D, DynamicState2D>(const DynamicState2D& src,
                                                                      const DynamicState2D& local_reference_frame,
                                                                      DynamicState2D& dst,
                                                                      const std::string& new_frame_name)
  {
    //HERE IT IS NEEDED A JACOBIAN
#pragma warning ("it is also needed to make the composition of the dynamic information. The dynamic information is not composed in this implementation")

    StateComposer::inverse_compose(src.pose, local_reference_frame.pose, dst.pose);
    //Supposing the local_reference_frame is static (not moving)
    dst.twist = src.twist;
  }

}

