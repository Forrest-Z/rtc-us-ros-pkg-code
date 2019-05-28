/*
 * collision_detection.h
 *
 *  Created on: Sep 13, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef COLLISION_DETECTION_H_
#define COLLISION_DETECTION_H_
#include <rtcus_navigation/navigation_component.h>

namespace rtcus_navigation
{

/**
 * \brief detects if a robot is in collision with the obstacles. Given the current
 * description of the obstacles and the shape of the robot.
 *
 * \remarks The approach is to consider always the collision in the robot local frame. In this
 * way the position of the robot is not needed. If a collision checker for the global frame is needed
 * Then use the GlobalFrameCollisionChecker.
 *
 * TODO: Try to integrate and make an implementation which uses the ROS FCL library.
 * */
template<typename ObstaclesType, typename RobotShapeType>
  class CollisionChecker : public NavigationNodeComponent
  {

  protected:
    CollisionChecker(NavigationNodeComponent& covariant_component) :
        NavigationNodeComponent::NavigationNodeComponent(covariant_component)
    {

    }
    CollisionChecker() :
        NavigationNodeComponent::NavigationNodeComponent(tCollisionChecker)
    {
    }
  public:

    virtual ~CollisionChecker()
    {
    }
    ;

    //TODO: Rename to check collision
    virtual bool detectCollision(const ObstaclesType& obstacles, const RobotShapeType& robot_shape) const=0;
    virtual void init()=0;
    virtual void reset()=0;

  };

}

#endif

