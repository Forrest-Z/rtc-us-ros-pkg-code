/*
 * configurable_shape.h
 *
 *  Created on: Dec 9, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef CONFIGURABLE_SHAPE_H_
#define CONFIGURABLE_SHAPE_H_

namespace rtcus_robot_shapes
{
template<typename ConfigType>
  class ConfigurableShape : public ConfigType
  {
  protected:

  public:
    ConfigurableShape()
    {
    }

    ConfigurableShape(const ConfigType& copy)
    {
      *this = copy;
      this->setConfig(copy);
    }

    virtual ~ConfigurableShape()
    {
    }

    virtual const ConfigType& setConfig(const ConfigType& config)=0;

  };
}

#endif /* CONFIGURABLE_SHAPE_H_ */
