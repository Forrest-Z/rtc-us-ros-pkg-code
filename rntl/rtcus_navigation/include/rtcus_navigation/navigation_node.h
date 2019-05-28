/*
 *
 *  Created on: Apr 6, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef NAVIGATION_NODE_H_
#define NAVIGATION_NODE_H_

#include <rtcus_navigation/abstract_navigation_node.h>
#include <rtcus_navigation/navigation_planner_base.h>
#include <rtcus_navigation/task_status.h>
#include <rtcus_navigation/state_estimation.h>
#include <rtcus_navigation/goal_port.h>
#include <rtcus_navigation/action_port.h>
#include <rtcus_navigation/reached_goal_predicate.h>
#include <rtcus_navigation/collision_checker.h>
#include <rtcus_navigation/state_port.h>
#include <rtcus_navigation/world_perception.h>
#include <rtcus_navigation/shape_model.h>
#include <rtcus_navigation/kinodynamic_model.h>

#include <rtcus_navigation/ControlCommand.h>
#include <rtcus_navigation/Status.h>
#include <rtcus_navigation/StatusPrediction.h>
#include <rtcus_navigation/StatusType.h>
#include <rtcus_navigation/state_estimation/state_publisher.h>

/**
 * \brief Compared with the the ROS navigation stack: This is a more generic framework. It does not uses costmaps as unique worldmodel.
 * The components "recovery behavior", "global planning" and "local planning" should be implemented through the NavigationPlanner Interface
 * since the robot is always in the origin of the coordinates frame, the goal pose has to be
 * periodically re-estimated based on the prediction expected of each taken action (twist cmd_vel)
 */
namespace rtcus_navigation
{

using namespace std;
using namespace rtcus_navigation;
using namespace boost;
using namespace rtcus_stamp;

/**
 *  \brief This template class represent the navigation node architecture for a move base node. This is composed by different architecture
 * components. The type representation of the different concepts required in a general navigation are template parameters so the architecture
 * is highly reusable and it also provide compile time architecture coherence checking.
 *
 * \tparam StateType This type describes the state of the robot. It could be a abstract type since the allocation is not made by this class.
 * \tparam ObstaclesType This type describes the perceiving surrounding world obstacles. It could be a abstract type since the allocation is not made by this class.
 * \tparam RoboShapeType represents the shape of the robot. Must be a non abstract type since Allocation is made inside this class.
 */
template<typename StateType, typename ObstaclesType, typename ActionType, typename GoalType, typename RobotShapeType,
    typename KinoDynamicDescription, typename TimeModel>
  class NavigationNode : public AbstractNavigationNode
  {

  public:
    USING_TIME_MODEL(TimeModel);

    /* CORE TYPE DEFINITIONS*/
    typedef TaskStatus<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription, TimeModel> TTaskStatus;
    typedef NavigationNode<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription,
        TimeModel> TNavigationNode;
    typedef NavigationPlanner<StateType, ObstaclesType, ActionType, GoalType, RobotShapeType, KinoDynamicDescription,
        TimeModel> TNavigationPlanner;

    /*DECLARING COMPONENT SHORT-TYPENAMES*/
    typedef StatePort<StateType, TimeModel> TStatePort;
    typedef StateEstimation<StateType, ActionType, TimeModel> TStateEstimation;
    typedef KinoDynamicModel<KinoDynamicDescription> TKinoDynamicModel;
    typedef ReachedGoalPredicate<StateType, GoalType> TReachedGoalPredicate;
    typedef GoalPort<GoalType, TimeModel> TGoalPort;
    typedef ActionPort<ActionType, TimeModel> TActionPort;
    typedef CollisionChecker<ObstaclesType, RobotShapeType> TCollisionChecker;
    typedef RobotShapeModel<RobotShapeType, TimeModel> TRobotShapeModel;
    typedef WorldPerceptionPort<ObstaclesType, TimeModel> TWorldPerceptionPort;

    /*TYPEDEFS FOR EXTERNAL REFERENCING FROM THIS CLASS*/
    typedef ObstaclesType TObstaclesType;
    typedef StateType TStateType;
    typedef ActionType TActionType;
    typedef GoalType TGoalType;
    typedef RobotShapeType TRobotShapeType;
    typedef KinoDynamicDescription TKinoDynamicDescription;
    typedef TimeModel TTimeModel;

    StatePublisher<StateType> state_estimation_pub_;

    //-------------- INTERNAL COMPONENTS ----------------------
  protected:

    //---- INPUTS ---

    /**
     * \brief How the robot see the obstacles
     * */
    shared_ptr<TWorldPerceptionPort> perception_port_;

    /**
     * \brief How the robot get and understand the goal
     * */
    shared_ptr<TGoalPort> goal_port_;

    /**
     * \brief How the robot perceive is own state
     * */
    shared_ptr<TStatePort> state_correction_port_;

    //OUTPUTS
    /**
     * \brief How the robot send an order to move
     * */
    shared_ptr<TActionPort> action_port_;

    //INTERNAL COMPONENTS
    shared_ptr<TRobotShapeModel> robot_shape_model_;
    shared_ptr<TKinoDynamicModel> kinodynamic_model_;
    shared_ptr<TNavigationPlanner> navigation_planner_;
    shared_ptr<TStateEstimation> state_estimation_;
    shared_ptr<TReachedGoalPredicate> reached_goal_predicate_;

    /**
     * TODO: This should be in the navigation planner if required. It shouldn't be in the navigation node.
     * In any case, use the type NoCollision Cheking if required.
     */
    shared_ptr<TCollisionChecker> collision_checker_;

    //----------- EVENT LIST --------------------
  public:
    typedef boost::signal<void(const TNavigationNode& sender, const TTaskStatus& current_status)> NavigationNodeEvent;

    /* \brief Its the same than onConfigUpdate but to get the typed node instead of the abstract node*/
    boost::signal<void(const TNavigationNode& sender)> onConfigurationUpdated;

    //TODO: try to convert this sender argument in const
    boost::signal<
        void(TNavigationNode& sender, const TTime& application_time, const ros::TimerEvent& time_info,
             const TTaskStatus& current_status)> onControlTaskBegin;

    NavigationNodeEvent onControlTaskEnd;

    NavigationNodeEvent onProcessingLocalWorldBegin;
    boost::signal<void(const TNavigationNode& sender, const ObstaclesType& obstacles)> onProcessingLocalWorldEnd;

    NavigationNodeEvent onProcessingLocalGoalBegin;
    boost::signal<void(const TNavigationNode& sender, const GoalType& resulting_goal, const TTime& application_time)> onProcessingLocalGoalEnd;

    NavigationNodeEvent onPlanningBegin;
    NavigationNodeEvent onPlaningEnd;

    NavigationNodeEvent onStateEstimationBegin;
    NavigationNodeEvent onStateEstimationEnd;

    NavigationNodeEvent onStateCorrectionBegin;
    NavigationNodeEvent onStateCorrectionEnd;
    NavigationNodeEvent onGoalReached;
    NavigationNodeEvent onGoalLost;

    boost::signal<void(const TNavigationNode& sender, const ActionType& eventArg, const TTime& applicationTime)> onPlanningResult;

    // ----------- PERIODIC NAVIGATION TASK INTERNAL STAGES -----------------------
  protected:

    void step_estimate_state(StampedData<StateType>& state_estimation, TTaskStatus& status);

    void step_estimate_goal(StampedData<GoalType>& resulting_goal, StampedData<StateType>& state_estimation,
                            TTaskStatus& status);

    void step_checking_goal_predicate(StampedData<StateType>& state_estimation, TTaskStatus& status);

    void step_process_local_world(Stamped<ObstaclesType>& resulting_obstacles, StampedData<StateType>& state_estimation,
                                  TTaskStatus& status);

    void step_check_collision(StampedData<StateType>& state_estimation, Stamped<ObstaclesType>& resulting_obstacles,
                              TTaskStatus& status);

    void step_planning(ActionType& cmd_vel, StampedData<StateType>& state_estimation,
                       StampedData<GoalType>& resulting_goal, Stamped<ObstaclesType>& resulting_obstacles,
                       TTaskStatus& status);

    void step_action_apply(ActionType& cmd_vel, TTaskStatus& status);

    void step_publish_status(TTaskStatus& status);

  public:
    NavigationNode();
    virtual ~NavigationNode();
    inline const TTaskStatus& getCurrentTaskStatus() const
    {
      return this->current_status_;
    }

    //------------------------ OVERRIDEN BASIC NODE WORKFLOW METHODS ------------------------------------------------------------------------------
  protected:
    virtual void onInit();
    virtual void onReset();
    virtual void onConfigurationUpdate(NavigationArchitectureConfig &config);
    virtual void controlTask(const ros::TimerEvent time);

    //------------------ GETTERS AND SETTERS FOR COMPONENTS ----------------------------------
  public:

    //WORLD PERCEPTION PORT
    void setWorldPerceptionPort(const shared_ptr<TWorldPerceptionPort>& perception_port);
    bool loadWorldPerceptionPort(const string& perception_port_name);
    inline shared_ptr<TWorldPerceptionPort> getWorldPerceptionPort() const
    {
      return perception_port_;
    }

    //GOAL PORT
    void setGoalPort(const shared_ptr<TGoalPort>& goal_port);
    bool loadGoalPort(const string& goal_port_name);
    inline shared_ptr<TGoalPort> getGoalPort() const
    {
      return goal_port_;
    }

    //ACTION PORT
    void setActionPort(const shared_ptr<TActionPort>& action_port);
    bool loadActionPort(const string& action_port);
    inline shared_ptr<TActionPort> getActionPort() const
    {
      return action_port_;
    }

    //NAVIGATION PLANNER
    void setNavigationPlanner(const shared_ptr<TNavigationPlanner>& navigation_planner);
    bool loadNavigationPlanner(const string& navigation_planner_name);
    inline shared_ptr<TNavigationPlanner> getNavigationPlanner() const
    {
      return navigation_planner_;
    }

    //COLLISION CHECKER
    void setCollisionChecker(const shared_ptr<TCollisionChecker>& collision_detection);

    template<class TInputCollisionChecker>
      void setCovariantCollisionChecker(const shared_ptr<TInputCollisionChecker>);

    bool loadCollisionChecker(const string& collision_checker_name);
    inline shared_ptr<TCollisionChecker> getCollisionChecker() const
    {
      return collision_checker_;
    }

    //REACH GOAL PREDICATE
    void setReachedGoalPredicate(const shared_ptr<TReachedGoalPredicate>& reach_goal_predicate);
    bool loadReachedGoalPredicate(const string& reach_goal_predicate_name);
    inline shared_ptr<TReachedGoalPredicate> getReachedGoalPredicate() const
    {
      return reached_goal_predicate_;
    }

    //STATE ESTIMATOR
    void setStateEstimation(const shared_ptr<TStateEstimation>& motion_model);
    bool loadStateEstimation(const string& motion_model_name);
    inline shared_ptr<TStateEstimation> getStateEstimation() const
    {
      return this->state_estimation_;
    }

    //STATE CORRECTION PORT
    void setStatePort(const shared_ptr<TStatePort>& state_port);
    bool loadStatePort(const string& state_port);
    inline shared_ptr<TStatePort> getStatePort() const
    {
      return this->state_correction_port_;
    }

    //SHAPE OF THE ROBOT
    void setRobotShapeModel(const shared_ptr<TRobotShapeModel>& shape_model);
    bool loadRobotShapeModel(const string& name);
    inline shared_ptr<TRobotShapeModel> getRobotShapeModel() const
    {
      return this->robot_shape_model_;
    }

    //KINODYNAMICS OF THE ROBOT
    void setKinodynamicModel(const shared_ptr<TKinoDynamicModel>& kinodynamic_model);
    bool loadKinodynamicModel(const string& name);
    inline shared_ptr<TKinoDynamicModel> getKinodynamicModel() const
    {
      return this->kinodynamic_model_;
    }

    //------------------- STATUS OF THE NODE --------------------------------------
  protected:
    /**
     * \brief Describes the current status of the navigation node. This is: the current stage it is in the moment and
     * also the partial computation and predictions of state, obstacles, shape and goal.
     * */
    TTaskStatus current_status_;
    bool last_goal_reached_detected;
    bool last_prediction_goal_reached_detected;

    /* \brief status_pub_ shows info about the known status based on corrected state and not prediction
     * */
    ros::Publisher status_pub_;

    /* \brief status_pub_ shows info about the known status based on the predicted state and not correction
     * */
    ros::Publisher status_prediction_pub_;
    void control_command_callback(const rtcus_navigation::ControlCommand::ConstPtr& msg);
    ros::Subscriber command_sub_;
  }
  ;
}
#endif

