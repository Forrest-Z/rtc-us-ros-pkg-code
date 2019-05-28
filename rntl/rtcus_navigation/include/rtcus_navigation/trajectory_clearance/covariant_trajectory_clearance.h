/*
 * covariant_collision_Checker.h
 *
 *  Created on: Dec 9, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef COVARIANT_TRAJECTORY_CLERANCE_H_
#define COVARIANT_TRAJECTORY_CLERANCE_H_

#include <rtcus_navigation/trajectory_clearance/trajectory_clearance.h>
#include <boost/shared_ptr.hpp>

namespace rtcus_navigation
{

using namespace boost;
using namespace rtcus_navigation::trajectory_clearance;

/*
 template<typename TSrc,typename TDest>
 struct ConvertPolicy
 {
 is_base_of<Base, Derived>::type is the type true_type.
 };

 */

template<template<typename, typename, typename, typename > class TTrajectoryClearanceSrc, typename SrcActionType,
    typename SrcObstaclesType, typename SrcRobotShape, typename SrcCostType, template<typename, typename, typename,
        typename > class TTrajectoryClearanceDst, typename DestActionType, typename DestObstaclesType,
    typename DestRobotShape, typename DestCostType>
  class CovariantTrajectoryClearance : public TTrajectoryClearanceDst<DestActionType, DestObstaclesType, DestRobotShape,
      DestCostType>
  {

  protected:
    shared_ptr<TTrajectoryClearanceSrc<SrcActionType, SrcObstaclesType, SrcRobotShape, SrcCostType> > decorated_;

  public:

    CovariantTrajectoryClearance(
        shared_ptr<TTrajectoryClearanceDst<SrcActionType, SrcObstaclesType, SrcRobotShape, SrcCostType> > decorated)
    {
      this->decorated_ = decorated;
      printf("decorating..\n");
    }

    virtual ~CovariantTrajectoryClearance()
    {
    }
    ;

    virtual bool computeClearance(const DestActionType& action, const DestObstaclesType& obstacles,
                                  const DestRobotShape& shape, DestCostType& clearance) const
    {
      return decorated_->computeClearance(action, obstacles, shape, clearance);
    }

    virtual void setMaxDistance(double distance)
    {
      decorated_->setMaxDistance(distance);
    }

    virtual double getMaxDistance() const
    {
      return decorated_->getMaxDistance();
    }

  };

//-----------------------------------------------------
template<template<typename, typename, typename, typename > class TTrajectoryClearanceDst, typename DestActionType,
    typename DestObstaclesType, typename DestRobotShape, typename DestCostType>
  class CovariantTraits<TTrajectoryClearanceDst<DestActionType, DestObstaclesType, DestRobotShape, DestCostType> >
  {
  public:
    typedef TTrajectoryClearanceDst<DestActionType, DestObstaclesType, DestRobotShape, DestCostType> TCovariantDst;
    template<template<typename, typename, typename, typename > class TTrajectoryClearanceSource, typename SrcActionType,
        typename SrcObstaclesType, typename SrcRobotShape, typename SrcCostType>
      static shared_ptr<TCovariantDst> CovariantDecorator(
          shared_ptr<TTrajectoryClearanceSource<SrcActionType, SrcObstaclesType, SrcRobotShape, SrcCostType> > src)
      {
        typedef TTrajectoryClearanceSource<SrcActionType, SrcObstaclesType, SrcRobotShape, SrcCostType> TSrc;
        typedef CovariantTrajectoryClearance<TTrajectoryClearanceSource, SrcActionType, SrcObstaclesType, SrcRobotShape,
            SrcCostType, TTrajectoryClearanceDst, DestActionType, DestObstaclesType, DestRobotShape, DestCostType> TCovariantType;
        return shared_ptr<TCovariantDst>((TCovariantDst*)new TCovariantType(static_pointer_cast<TSrc>(src)));
      }

  };

/**
 * This could be needed if the InputType has several abstract interpretations (multiple inheritance of interface exteding from TrajectoryClearance). This
 * allows to set explicitly which one of them it has to be used
 * */
/*template<typename TSrc, typename TDest>
 class ExplicitCovarianceTraits
 {
 public:
 template<typename InputType>
 static boost::shared_ptr<TDest> CovariantDecorator(boost::shared_ptr<InputType> input)
 {
 return rtcus_navigation::CovariantTraits<TDest>::CovariantDecorator(boost::static_pointer_cast<TSrc>(input));
 }
 };
 */
}

#endif /* COVARIANT_COLLISION_Checker_H_ */
