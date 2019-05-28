/*
 * lookup_table_test.cpp
 *
 *  Created on: Dec 12, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#include <mrpt/utils/CTicTac.h>
#include <gtest/gtest.h>
#include <rtcus_navigation/trajectory_clearance/circular_trajectory_clearance_circular_robot.h>
#include <rtcus_navigation/trajectory_clearance/trajectory_clearance_lookup_table_decorator.h>
#include <rtcus_robot_shapes/circular_robot.h>
#include <rtcus_nav_msgs/Twist2D.h>
#include <boost/make_shared.hpp>
#include <rtcus_kinodynamic_description/NonHolonomicKinoDynamicsConfig.h>
#include <boost/timer.hpp>
#include <rtcus_robot_shapes/circular_robot.h>
#include <rtcus_assert/rtcus_assert.h>

using namespace rtcus_navigation::trajectory_clearance;
using namespace rtcus_robot_shapes::interfaces;
using namespace rtcus_nav_msgs;

/*configuration*/
rtcus_kinodynamic_description::NonHolonomicKinoDynamicsConfig kinodynamic_desc;
bool allow_slow_computations = true;

//---------------------------------------------------------------
float workspace_distance = 40;
unsigned int workspace_resolution = 100;
unsigned int action_count = 301;
static float knorm = (float)(action_count - 1) / M_PI_2;
static const float NUMERICAL_MIN = 0.0002;

//this will be called in real time
int get_index_from_action(const Twist2D& action)
{
  if (action.angular < NUMERICAL_MIN)
    return action_count / 2;
  else
  {
    double alpha = atan2(action.linear, action.angular);
    //symetry
    unsigned int action_index = fabs(knorm * alpha) + 1;
    return action_index;
  }
}
//this is just for the generation of the lookup table
void get_action_from_index(unsigned int action_index, Twist2D& action)
{
  float alpha = (M_PI_2 / (action_count)) * action_index;
  action.linear = sin(alpha);
  action.angular = cos(alpha);
}

//this will be called in real time
int get_constant_index_from_action(const Twist2D& action)
{
  return 0;
}
//---------------------------------------------------------------------------

double loop(
    boost::shared_ptr<TrajectoryClearance<Twist2D, pcl::PointXY, ICircularRobotShape, DefaultClearanceCost> > clearance_method,
    const ICircularRobotShape& robot_shape)
{
  //-----------------------------------------------------------
  Twist2D current_action;
  current_action.linear = 0;

  double cumulated_time_computation = 0;

  printf("\nSTARTING EVALUATION\n");
  int cont_action = 0;
  long test_count = 0;
  unsigned int v_samples = 10;
  unsigned int omega_samples = 10;
  mrpt::utils::CTicTac clock;
  boost::timer t;
  t.restart();
  double res = 0;

  for (unsigned int i = 0; i < v_samples; i++)
  {
    current_action.linear = ((float)i / (float)v_samples) * kinodynamic_desc.linear_forward_speed_limit
        - kinodynamic_desc.linear_backwards_speed_limit;
    for (unsigned int j = 0; j < omega_samples; j++)
    {
      current_action.angular = (((float)j / (float)omega_samples) * 2 * kinodynamic_desc.angular_speed_limit)
          - kinodynamic_desc.angular_speed_limit;
      //printf("action %d", cont_action++);

      unsigned int index = get_index_from_action(current_action);
      RTCUS_ASSERT_MSG(index >= 0 && index < action_count, "current action index is not correct: %d for action %lf %lf",
                       index, current_action.linear, current_action.angular);

      for (float x = 5; x < 10.0; x += 0.1)
        for (float y = -10.0; y < 10.0; y += 0.1)
        {
          pcl::PointXY obstacle;
          obstacle.x = x;
          obstacle.y = y;
          //printf("\n[obst %lf %lf]", obstacle.x, obstacle.y);
          ROS_INFO("CurrentTest: %d", test_count++);

          DefaultClearanceCost clearance_computed;

          double min_delta_t = std::numeric_limits<double>::max();
          for (int l = 0; l < 10; l++)
          {
            clock.Tic();
            bool computed_clearance_ok = clearance_method->computeClearance(current_action, obstacle, robot_shape,
                                                                            clearance_computed);
            double delta_t = clock.Tac();
            if (delta_t < min_delta_t)
              min_delta_t = delta_t;
          }
          res += min_delta_t;
          test_count++;
        }
    }
  }

  //cumulated_time_computation = t.elapsed();
  //std::cout <<"timer results: "<<clock;

  return res;
}

TEST(LookupTables, BasicSample)
{
  kinodynamic_desc.linear_forward_speed_limit = 20; // m/sec
  kinodynamic_desc.linear_backwards_speed_limit = 0; // m/sec
  kinodynamic_desc.angular_speed_limit = 1; //m/sec

  boost::shared_ptr<TrajectoryClearance<Twist2D, pcl::PointXY, ICircularRobotShape, DefaultClearanceCost> > decorated_clearance_method =
      boost::make_shared<CircularTrajectoryClearanceForCircularRobot>();

  rtcus_robot_shapes::CircularRobot robot_shape;
  robot_shape.radius = 4.5;

//----------------------------------------------------------
  printf("GENERATING THE LOOKUP TABLE...\n");
  boost::timer t;

  boost::shared_ptr<TrajectoryClearanceLookupTable2D<Twist2D, ICircularRobotShape> > lookup_table = boost::shared_ptr<
      TrajectoryClearanceLookupTable2D<Twist2D, ICircularRobotShape> >(
      new TrajectoryClearanceLookupTable2D<Twist2D, ICircularRobotShape>(decorated_clearance_method, workspace_distance,
                                                                         workspace_resolution, action_count,
                                                                         get_index_from_action, get_action_from_index,
                                                                         allow_slow_computations));

  RTCUS_ASSERT_MSG(lookup_table->checkAcctionIndexMapping(), "Given functions for indexing actions are not coherent");
  lookup_table->precomputeLookupTable(robot_shape);

  printf("\nEND GENERATION OF THE LOOKUP TABLE. Time (%lf)\n", t.elapsed());

  double computed_time = loop(decorated_clearance_method, robot_shape);
  double lookup_time = loop(lookup_table, robot_shape);

  printf("\???nConstant index accessor\n===");
  lookup_table->setIndexFromActionFunction(get_constant_index_from_action);
  lookup_table->precomputeLookupTable(robot_shape);

  double lookup_time_constant = loop(lookup_table, robot_shape);

  //printf("Total of expermients: %ld\n\n", test_count);
  printf("- Total cummulated computed clearance: %lf\n", computed_time);
  printf("- Total cummulated lookup clearance: %lf\n", lookup_time);
  printf("- Total cummulated lookup clearance: %lf\n", lookup_time_constant);

  //CHECK PERFORMANCE HERE
}

