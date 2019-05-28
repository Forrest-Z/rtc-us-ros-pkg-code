/*
 * covariant_collision_Checker.h
 *
 *  Created on: Dec 9, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef COVARIANT_COLLISION_Checker_H_
#define COVARIANT_COLLISION_Checker_H_

#include <rtcus_navigation/collision_checker.h>
#include <boost/shared_ptr.hpp>

namespace rtcus_navigation
{

using namespace boost;

template<typename SrcCollisionChekr, typename DstCollisionChecker>
  class CovariantCollisionChecker
  {
  };

template<template<typename, typename > class TCollisionCheckerSrc, typename SrcObstaclesType, typename SrcRobotShape,
    template<typename, typename > class TCollisionCheckerDst, typename DestObstaclesType, typename DestRobotShape>
  class CovariantCollisionChecker<TCollisionCheckerSrc<SrcObstaclesType, SrcRobotShape>,
      TCollisionCheckerDst<DestObstaclesType, DestRobotShape> > : public TCollisionCheckerDst<DestObstaclesType,
      DestRobotShape>
  {

  protected:
    boost::shared_ptr<TCollisionCheckerSrc<SrcObstaclesType, SrcRobotShape> > decorated_;

  public:
    CovariantCollisionChecker(boost::shared_ptr<TCollisionCheckerDst<SrcObstaclesType, SrcRobotShape> > decorated)
    {
      this->decorated_ = decorated;
    }
    inline ~CovariantCollisionChecker()
    {
    }
    ;

    inline bool detectCollision(const DestObstaclesType& obstacles, const DestRobotShape& robot_shape) const
    {
      return decorated_->detectCollision(obstacles, robot_shape);
    }
    inline void init()
    {
      decorated_->init();
    }
    inline void reset()
    {
      decorated_->reset();
    }
  };


//-----------------------------------------------------------------------
template<template<typename, typename > class TCollisionCheckerDst, typename DestObstaclesType, typename DestRobotShape>
  class CovariantTraits<TCollisionCheckerDst<DestObstaclesType, DestRobotShape> >
  {
  public:
    typedef TCollisionCheckerDst<DestObstaclesType, DestRobotShape> TCovariantDst;
    template<template<typename, typename > class TCollisionCheckerSource, typename SrcObstaclesType,
        typename SrcRobotShape>
      static void CovariantDecorator(boost::shared_ptr<TCollisionCheckerSource<SrcObstaclesType, SrcRobotShape> > src)
      {
        return;
      }

    template<typename TDerivedCollisionChecker>
      static boost::shared_ptr<TCovariantDst> CovariantDecorator(boost::shared_ptr<TDerivedCollisionChecker> src)
      {

        typedef typename TDerivedCollisionChecker::TCollisionChecker TSrcCollisionChecker;
        shared_ptr<TSrcCollisionChecker> compatible_ptr = dynamic_pointer_cast<TSrcCollisionChecker>(src);

        return boost::shared_ptr<TCovariantDst>(
            new CovariantCollisionChecker<TSrcCollisionChecker, TCovariantDst>(compatible_ptr));
      }
  };

}

#endif /* COVARIANT_COLLISION_Checker_H_ */
