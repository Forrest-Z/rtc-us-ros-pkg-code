/*
 * dynamic_2D_state_port.h
 *
 *  Created on: Apr 10, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef DYNAMIC_2D_STATE_PORT_H_
#define DYNAMIC_2D_STATE_PORT_H_

#include <rtcus_navigation/state_ports/adaptable_state_correction_port.h>
#include <nav_msgs/Odometry.h>
#include <rtcus_nav_msgs/DynamicState.h>
#include <tf/tf.h>

namespace rtcus_navigation
{
namespace state_ports
{
class DynamicState2DFromOdometryPort : public AdaptableStateCorrectionPortBase<DynamicState2D, Odometry>
{
protected:
  bool first_time;
  virtual void fromInputRosMsgToInternalState(const Odometry& msg, DynamicState2D& output_state,
                                              ros::Time& output_stamp, std::string& frame_id)
  {
    StampedData<DynamicState2D> last_state;
    this->getLastState(last_state);

    nav_msgs::Odometry odom_with_twist_estimation;
    if (!first_time)
    {
      odom_with_twist_estimation.header = msg.header;

      tf::Quaternion last_state_quaternion = tf::createQuaternionFromYaw(last_state.getConstData().pose.phi);
      tf::Transform last_state_transform(
          last_state_quaternion, tf::Vector3(last_state.getConstData().pose.x, last_state.getConstData().pose.y, 0));

      ROS_INFO(
          "%s. past state transformation pos [%lf %lf] rot %lf [%lf %lf %lf %lf]", typeid(this).name(), last_state_transform.getOrigin().x(), last_state_transform.getOrigin().y(), tf::getYaw(last_state_quaternion), last_state_quaternion.x(), last_state_quaternion.y(), last_state_quaternion.z(), last_state_quaternion.w());
      tf::Quaternion current_state_quaternion;
      tf::Point current_state_position;
      tf::quaternionMsgToTF(msg.pose.pose.orientation, current_state_quaternion);
      tf::pointMsgToTF(msg.pose.pose.position, current_state_position);
      tf::Transform current_state_transform(current_state_quaternion, current_state_position);

      ROS_INFO(
          "%s. current msg transformation pos [%lf %lf] rot %lf [%lf %lf %lf %lf]", typeid(this).name(), current_state_transform.getOrigin().x(), current_state_transform.getOrigin().y(), tf::getYaw(current_state_transform.getRotation()), current_state_quaternion.x(), current_state_quaternion.y(), current_state_quaternion.z(), current_state_quaternion.w());
      tf::Transform delta_transform = current_state_transform * last_state_transform.inverse();

      //en la práctica el algoritmo dwa es muy sensible a la lectura de la velocidad actual. Esto es
      //así especialmente cuando el periodo de evitación de obstáculos es pequeño y que como consecuencia
      //define una ventana dinámica muy. Dado que el algoritmo coquetea en los límites de las velocidades
      //admisibles puede verse de forma repentina aislado y bloqueado en una ventana sin ninguna
      //velocidad admisible

      ros::Duration delta_time = msg.header.stamp - last_state.getStamp();

      ROS_INFO(
          "delta transform %lf %lf %lf", delta_transform.getOrigin().x(), delta_transform.getOrigin().y(), tf::getYaw(delta_transform.getRotation()));

      odom_with_twist_estimation.pose = msg.pose;
      odom_with_twist_estimation.twist.twist.linear.x = delta_transform.getOrigin().x() / delta_time.toSec();
      odom_with_twist_estimation.twist.twist.linear.y = delta_transform.getOrigin().y() / delta_time.toSec();
      odom_with_twist_estimation.twist.twist.linear.z = 0.0;

      odom_with_twist_estimation.twist.twist.angular.z = tf::getYaw(delta_transform.getRotation()) / delta_time.toSec();
      odom_with_twist_estimation.twist.twist.angular.x = 0.0;
      odom_with_twist_estimation.twist.twist.angular.y = 0.0;

      ROS_INFO(
          "Odometry twist [%lf %lf %lf] vs Estimated Twist [%lf %lf %lf]", msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z, odom_with_twist_estimation.twist.twist.linear.x, odom_with_twist_estimation.twist.twist.linear.y, odom_with_twist_estimation.twist.twist.angular.z);

    }
    else
    {
      odom_with_twist_estimation = msg;
      first_time = false;
      ROS_INFO("state port, first state msg");
    }

    rtcus_conversions::Conversions::convert(odom_with_twist_estimation, output_state);

    if (isHeadedRosMsg(odom_with_twist_estimation, output_stamp, frame_id))
    {
      setHeaderRosMsg(output_state, output_stamp, frame_id);
    }
    else
    {
#define msg_error "State Correction Port. This state port implementation does not support input msgs without header"
      ROS_ERROR(msg_error);
      throw ros::Exception(msg_error);
    }

  }
public:
  DynamicState2DFromOdometryPort() :
      first_time(true)
  {

  }
  virtual void reset()
  {
    AdaptableStateCorrectionPortBase<DynamicState2D, Odometry>::reset();
    first_time = true;
  }
  virtual ~DynamicState2DFromOdometryPort()
  {
  }

};
}
;
}
;

#endif /* DYNAMIC_2D_STATE_PORT_H_ */
