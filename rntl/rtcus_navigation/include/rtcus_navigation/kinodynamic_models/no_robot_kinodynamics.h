/*
 * no_robot_shape.h
 *
 *  Created on: Dec 8, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef NO_ROBOT_KINO_H_
#define NO_ROBOT_KINO_H_

#include <rtcus_navigation/kinodynamic_model.h>

namespace rtcus_navigation
{
namespace kinodynamic_models
{
class NoRobotShapeModel : public rtcus_navigation::KinoDynamicModel<void>
{
public:
  virtual ~NoRobotShapeModel()
  {
  }
  virtual void init()
  {

  }

  virtual void getKinodynamics(void& knds) const
  {

  }
  virtual void reset()
  {
  }
};
}
}

#endif /* NO_ROBOT_SHAPE_H_ */
