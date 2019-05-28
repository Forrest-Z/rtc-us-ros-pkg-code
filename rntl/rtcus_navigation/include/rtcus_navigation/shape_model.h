/*
 *  Created on: Dec 8, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef ROBOT_SHAPE_DESCRIPOR_H_
#define ROBOT_SHAPE_DESCRIPOR_H_

#include <rtcus_navigation/core.h>

namespace rtcus_navigation
{

template<typename RobotShapeType, typename TimeModel= ROSTimeModel>
  class RobotShapeModel : public NavigationNodeComponent
  {
    USING_TIME_MODEL(TimeModel);

    /**
     * \brief Protected constructor to force the specialization
     * */
  protected:
    RobotShapeModel():NavigationNodeComponent(tShapeModel)
    {

    }

  public:
    virtual void reset()=0;
    virtual void init()=0;
    /**
     * \brief Should return the shape of the robot at the current moment.
     * */
    virtual void getRobotShape(RobotShapeType& shape) const=0;

    /**
     * \brief If it is allowed, the shape of the robot can be modified programatically using this method
     */
    virtual void setRobotShape(const RobotShapeType& shape)=0;

    /**
     * \brief Should return the shape of the robot in the specified moment. Typically for static shapes the default behavior
     * is redirecting this method to the method getRobotShape
     * return the current robot shape (expected to hold the same shape in the future)
     * */
    virtual void predictRobotShape(TTime time,  RobotShapeType& shape)=0;

    /**
     *  \ brief This method should be called by the implementation when new changes in the robot happens.
     * */
    boost::signal<void(const RobotShapeType&)> onShapeChanged;
    virtual ~RobotShapeModel()
    {
    }

  };
}

#endif /* ROBOT_SHAPE_DESCRIPOR_H_ */
