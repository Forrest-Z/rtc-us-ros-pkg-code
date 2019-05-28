/*
 *
 *  Created on: Jun 19, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef STATE_ESTIMATION_TF_PUBLISHER_H
#define STATE_ESTIMATION_TF_PUBLISHER_H

#include <rtcus_navigation/navigation_node.h>
#include <rtcus_navigation/state_estimation/default_state_estimation.h>
#include <rtcus_conversions/conversions.h>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/foreach.hpp>
#include <rtcus_navigation_tools/visual_representations/goal_representation.h>
#include <rtcus_motion_models/motion_models/motion_models_impl.h>

namespace rtcus_navigation_tools
{
using namespace rtcus_navigation;
using namespace rtcus_navigation::state_estimation;
using namespace boost;

template<class TNavigationNode>
  class TFPublisher
  {
    USING_NAVIGATION_TYPES(TNavigationNode)
    ;

  protected:
    shared_ptr<TNavigationNode> navigation_navigation_node_;
    bool last_prediction_trajectory_paint_;

    ros::NodeHandle nh_;
    ros::Publisher trajectory_markers_pub_;
    ros::Publisher states_markers_pub_;
    ros::Publisher past_states_trajectory_markers_pub_;
    ros::Publisher state_estimation_history_pub_;
    rtcus_navigation_tools::visual_representations::GoalVisualRepresentation<TGoalType> goal_visual_representation_;

    void publish_goal(const TNavigationNode& nn, const TGoalType& estimated_goal, const ros::Time application_time)
    {
      goal_visual_representation_.represent(estimated_goal);
    }

    void publish_trajectory_estimation(const TNavigationNode& nn, const TTaskStatus& status)
    {
      shared_ptr<DefaultStateEstimation<TStateType, TActionType, ROSTimeModel> > est_ptr = dynamic_pointer_cast<
          DefaultStateEstimation<TStateType, TActionType, ROSTimeModel> >(nn.getStateEstimation());

      if (est_ptr && est_ptr->getStateHistory().size() > 0)
      {
        DefaultStateEstimation<TStateType, TActionType, ROSTimeModel>& est = *est_ptr;

        // ------------------- 1 - PAINT CURRENT STATE_HISTORY -----------------------------------------------
        {
          visualization_msgs::Marker state_history_line_strip;
          //TODO: generalize this tf frame
          state_history_line_strip.header.frame_id = navigation_navigation_node_->getReferenceFrame();
          state_history_line_strip.header.stamp = ros::Time();
          state_history_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
          state_history_line_strip.action = visualization_msgs::Marker::ADD;
          state_history_line_strip.ns = "past_state_trajectory";
          state_history_line_strip.id = 0;
          state_history_line_strip.scale.x = 0.02;
          state_history_line_strip.scale.y = 0.005;
          state_history_line_strip.scale.z = 0.005;
          state_history_line_strip.pose.orientation.w = 1.0;

          state_history_line_strip.color.b = 1.0;
          state_history_line_strip.color.g = 0.2;
          state_history_line_strip.color.r = 0.2;
          state_history_line_strip.color.a = 1.0;
          state_history_line_strip.lifetime = ros::Duration(10.0);

          //TODO: Change this to geometry_msgs/Path
          const list<StampedData<TStateType> >& state_trajectory = est.getStateHistory();
          BOOST_FOREACH(const StampedData<TStateType> & state, state_trajectory)
          {
            geometry_msgs::Point p;
            p.x = state->pose.x;
            p.y = state->pose.y;
            state_history_line_strip.points.push_back(p);
          }
          this->past_states_trajectory_markers_pub_.publish(state_history_line_strip);
        }

        // --------------------------2 - PAINT PREDICTION HISTORY -----------------------------------------------------------
        {
          visualization_msgs::Marker prediction_history_line_strip;
          //TODO: generalize this tf frame
          prediction_history_line_strip.header.frame_id = navigation_navigation_node_->getReferenceFrame();
          prediction_history_line_strip.header.stamp = ros::Time();
          prediction_history_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
          prediction_history_line_strip.action = visualization_msgs::Marker::ADD;
          prediction_history_line_strip.ns = "state_prediction_history";
          prediction_history_line_strip.id = 0;
          prediction_history_line_strip.scale.x = 0.02;
          prediction_history_line_strip.scale.y = 0.005;
          prediction_history_line_strip.scale.z = 0.005;
          prediction_history_line_strip.pose.orientation.w = 1.0;

          prediction_history_line_strip.color.b = 0.8;
          prediction_history_line_strip.color.g = 0.8;
          prediction_history_line_strip.color.r = 1.0;
          prediction_history_line_strip.color.a = 1.0;
          prediction_history_line_strip.lifetime = ros::Duration(10.0);

          //-----------------------------------------------

          typedef typename DefaultStateEstimation<TStateType, TActionType, ROSTimeModel>::PreviousStateEstimation PreviousStateEstimation;
          typedef typename list<PreviousStateEstimation>::const_iterator StateEstimationIterator;
          const list<PreviousStateEstimation>& state_estimation_history = est.getStateEstimationHistory();
          //TODO: Change this to geometry_msgs/Path
          for (StateEstimationIterator it = state_estimation_history.begin(); it != state_estimation_history.end();
              it++)
          {
            const PreviousStateEstimation& state = *it;
            geometry_msgs::Point p;
            p.x = state.state_estimation->pose.x;
            p.y = state.state_estimation->pose.y;
            prediction_history_line_strip.points.push_back(p);
          }
          this->state_estimation_history_pub_.publish(prediction_history_line_strip);
        }

        // ----------------------------- 3 - PAINT CURRENT PREDICTION TRAJECTORY ----------------------------------------
        if (est.getNonConfirmedActions().size() > 0)
        {
          StampedData<TStateType> last_state_reading = est.getStateHistory().back(); //begin
          StampedData<TStateType> last_state_estimation; //end
          est.getLastStateEstimation(last_state_estimation);

          vector<StampedData<TActionType> > actions;
          est.estimateState(last_state_reading, last_state_estimation.getStamp(), actions);

          ROS_DEBUG("Using motion model: sampling trajectory.");

          visualization_msgs::Marker line_strip;
          //TODO: generalize this tf frame
          line_strip.header.frame_id = navigation_navigation_node_->getReferenceFrame();
          line_strip.header.stamp = ros::Time();
          line_strip.type = visualization_msgs::Marker::LINE_STRIP;

          line_strip.ns = "prediction_trajectory";

          if (actions.size() == 0)
          {
            if (last_prediction_trajectory_paint_)
            {
              line_strip.action = visualization_msgs::Marker::DELETE;
              last_prediction_trajectory_paint_ = false;
              this->trajectory_markers_pub_.publish(line_strip);
            }
          }
          else
          {
            last_prediction_trajectory_paint_ = true;
            if (last_prediction_trajectory_paint_)
              line_strip.action = visualization_msgs::Marker::MODIFY;
            else
              line_strip.action = visualization_msgs::Marker::ADD;

            //avoiding asserts and numerical instabilities
            if (actions.back().getStamp() > last_state_estimation.getStamp())
            {
              ROS_ERROR("THIS should not happen");
              actions.back().setStamp(last_state_estimation.getStamp());
            }
            actions.front().setStamp(last_state_reading.getStamp());

            ROS_DEBUG_STREAM("start state:" << last_state_reading.getData());
            vector<StampedData<TStateType> > trajectory(100);
            est.getMotionModel()->sampleStates(last_state_reading, actions, last_state_estimation.getStamp(),
                                               trajectory);

            ROS_DEBUG_STREAM("trajectory stare:" << trajectory.front().getData());

            line_strip.id = 0;
            line_strip.scale.x = 0.02;
            line_strip.scale.y = 0.005;
            line_strip.scale.z = 0.005;
            line_strip.pose.orientation.w = 1.0;

            line_strip.color.b = 0.3;
            line_strip.color.g = 0.3;
            line_strip.color.r = 1.0;
            line_strip.color.a = 1.0;
            line_strip.lifetime = ros::Duration(10.0);
            //TODO: Change this to geometry_msgs/Path

            for (unsigned int i = 0; i < trajectory.size(); i++)
            {
              geometry_msgs::Point p;
              p.x = trajectory[i]->pose.x;
              p.y = trajectory[i]->pose.y;

              line_strip.points.push_back(p);
            }
            this->trajectory_markers_pub_.publish(line_strip);
          }
        }
      }
    }

    void paint_state_markers_aux(visualization_msgs::MarkerArray& state_markers, const std::string& reference_frame,
                                 std_msgs::ColorRGBA color)
    {
      visualization_msgs::Marker m;
      m.header.frame_id = reference_frame;
      m.ns = reference_frame;

      m.header.stamp = ros::Time();
      m.type = visualization_msgs::Marker::CUBE;
      m.action = visualization_msgs::Marker::ADD;
      m.id = 0;
      m.scale.x = 0.08;
      m.scale.y = 0.04;
      m.scale.z = 0.04;
      m.pose.orientation.w = 1.0;

      m.color = color;
      m.lifetime = ros::Duration(10.0);
      state_markers.markers.push_back(m);

    }

    void publish_ptc_state_markets(const TNavigationNode& nn, const TTaskStatus& status)
    {
      visualization_msgs::MarkerArray state_markers;

      std_msgs::ColorRGBA color;
      color.b = 1.0;
      color.g = 0.2;
      color.r = 1.0;
      color.a = 1.0;
      paint_state_markers_aux(state_markers, "state_at_last_obstacle_reading", color);

      color.b = 0.2;
      color.g = 0.2;
      color.r = 1.0;
      color.a = 1.0;
      paint_state_markers_aux(state_markers, nn.getBasePredictionFrame(), color);

      color.b = 0.2;
      color.g = 1.0;
      color.r = 1.0;
      color.a = 1.0;
      paint_state_markers_aux(state_markers, nn.getBaseFrameName(), color);

      color.b = 1.0;
      color.g = 0.2;
      color.r = 0.2;
      color.a = 1.0;
      paint_state_markers_aux(state_markers, "last_state_reading", color);

      states_markers_pub_.publish(state_markers);

    }

    tf::TransformBroadcaster broadcaster_;
    void publish_state_at_obstacle_reading(const TNavigationNode& nn, const TTaskStatus& status)
    {
      const StampedData<TStateType>& state_at_obstacle_reading_ = status.getStateAtLastObstacleReading();
      tf::StampedTransform transf;

      transf.stamp_ = ros::Time::now();
      transf.frame_id_ = nn.getReferenceFrame();
      transf.child_frame_id_ = "state_at_last_obstacle_reading";

      rtcus_conversions::Conversions::convert(state_at_obstacle_reading_.getConstData(), (tf::Transform&)transf);
      broadcaster_.sendTransform(transf);
    }

    void publish_estimation(const TNavigationNode& nn, const TTaskStatus& status)
    {
      const StampedData<TStateType>& last_state_estimation_ = status.getStateEstimation();
      tf::StampedTransform transf;

      transf.stamp_ = ros::Time::now();
      transf.frame_id_ = nn.getReferenceFrame();
      transf.child_frame_id_ = nn.getBasePredictionFrame();

      rtcus_conversions::Conversions::convert(last_state_estimation_.getConstData(), (tf::Transform&)transf);
      broadcaster_.sendTransform(transf);
    }

    void publish_state_at_state_reading(const TNavigationNode& nn, const TTaskStatus& status)
    {
      const Stamped<TStateType>& last_state_reading = status.getLastStateReading();
      tf::StampedTransform transf;

      transf.stamp_ = ros::Time::now();
      transf.frame_id_ = navigation_navigation_node_->getReferenceFrame();
      transf.child_frame_id_ = "last_state_reading";

      rtcus_conversions::Conversions::convert(last_state_reading.getConstData(), (tf::Transform&)transf);
      broadcaster_.sendTransform(transf);
    }

  public:
    TFPublisher(const shared_ptr<TNavigationNode>& navigation_node) :
        navigation_navigation_node_(navigation_node), nh_(
            navigation_node->getPrivateNode().getNamespace() + "/visual_representation"), goal_visual_representation_(
            navigation_node->getBasePredictionFrame(), nh_)
    {

      last_prediction_trajectory_paint_ = false;
      trajectory_markers_pub_ = nh_.advertise<visualization_msgs::Marker>("prediction_trajectory", 0);
      states_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("ptc_states", 0);
      past_states_trajectory_markers_pub_ = nh_.advertise<visualization_msgs::Marker>("past_states_trajectory", 0);
      state_estimation_history_pub_ = nh_.advertise<visualization_msgs::Marker>("state_estimation_history", 0);

      navigation_node->onControlTaskEnd.connect(
          bind(&TFPublisher<TNavigationNode>::publish_state_at_obstacle_reading, this, _1, _2));
      navigation_node->onControlTaskEnd.connect(
          bind(&TFPublisher<TNavigationNode>::publish_state_at_state_reading, this, _1, _2));
      navigation_node->onControlTaskEnd.connect(bind(&TFPublisher<TNavigationNode>::publish_estimation, this, _1, _2));

      navigation_node->onControlTaskEnd.connect(
          bind(&TFPublisher<TNavigationNode>::publish_trajectory_estimation, this, _1, _2));
      navigation_node->onControlTaskEnd.connect(
          bind(&TFPublisher<TNavigationNode>::publish_ptc_state_markets, this, _1, _2));

      //navigation_node->getStatePort()->onStateReceived.connect(bind(&TFPublisher::publish_state_at_state_reading, this, _1, _2));
      navigation_node->onProcessingLocalGoalEnd.connect(
          bind(&TFPublisher<TNavigationNode>::publish_goal, this, _1, _2, _3));
    }
  };

}

#endif /* TRAJECTORY_PUBLISHER_H_ */
