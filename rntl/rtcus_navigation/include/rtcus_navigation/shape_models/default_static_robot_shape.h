/*
 * default_static_robot_shape.h
 *
 *  Created on: Dec 9, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef DEFAULT_STATIC_ROBOT_SHAPE_H_
#define DEFAULT_STATIC_ROBOT_SHAPE_H_

#include <rtcus_navigation/shape_model.h>

namespace rtcus_navigation
{
namespace shape_models
{
template<typename RobotShapeType, typename TimeModel = ROSTimeModel>
  class DefaultStaticRobotShape : public rtcus_navigation::RobotShapeModel<RobotShapeType, TimeModel>
  {
    USING_TIME_MODEL(TimeModel);
  protected:
    RobotShapeType data_;
  public:
    virtual ~DefaultStaticRobotShape()
    {
    }
    virtual void init()
    {
    }

    virtual void setRobotShape(const RobotShapeType& shape)
    {
      *((RobotShapeType*)(&this->data_)) = shape;
      this->onShapeChanged(this->data_);
    }

    virtual void getRobotShape(RobotShapeType& shape) const
    {
      shape = this->data_;
    }

    /**
     *  \brief static robot shape. The time does not matter.
     * */
    virtual void predictRobotShape(TTime time, RobotShapeType& shape)
    {
      getRobotShape(shape);
    }

    virtual void reset()
    {
    }
  };
}
}
#endif /* DEFAULT_STATIC_ROBOT_SHAPE_H_ */
