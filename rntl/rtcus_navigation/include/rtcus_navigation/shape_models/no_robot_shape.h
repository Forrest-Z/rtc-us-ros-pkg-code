/*
 * no_robot_shape.h
 *
 *  Created on: Dec 8, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef NO_ROBOT_SHAPE_H_
#define NO_ROBOT_SHAPE_H_

#include <rtcus_navigation/shape_model.h>

namespace rtcus_navigation
{
namespace shape_models
{
class NoRobotShapeModel : public rtcus_navigation::RobotShapeModel<void, ROSTimeModel>
{
public:
  virtual ~NoRobotShapeModel()
  {
  }
  virtual void init()
  {

  }

  virtual void getRobotShape(void& shape) const
  {

  }

  virtual void predictRobotShape(TTime time, const void& shape)
  {

  }

  virtual void reset()
  {
  }
};
}
}

#endif /* NO_ROBOT_SHAPE_H_ */
