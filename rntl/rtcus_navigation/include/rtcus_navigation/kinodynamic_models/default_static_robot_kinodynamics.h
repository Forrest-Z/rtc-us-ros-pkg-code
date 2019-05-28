/*
 * default_static_robot_shape.h
 *
 *  Created on: Dec 9, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef DEFAULT_KINODYNAMICMODEL_H_
#define DEFAULT_KINODYNAMICMODEL_H_

#include <rtcus_navigation/kinodynamic_model.h>

namespace rtcus_navigation
{
namespace kinodynamic_models
{
template<typename KinoDynamicDescription>
  class DefaultKinodynamicModel : public rtcus_navigation::KinoDynamicModel<KinoDynamicDescription>
  {
  protected:
    KinoDynamicDescription data_;
  public:
    virtual ~DefaultKinodynamicModel()
    {
    }
    virtual void init()
    {
    }

    virtual void setKinoDynamics(const KinoDynamicDescription& knds)
    {
      this->data_ = knds;
      this->onKinodynamicsChanged(this->data_);
    }

    virtual void getKinodynamics(KinoDynamicDescription& knds) const
    {
      knds = this->data_;

    }

    virtual void reset()
    {
    }
  };
}
}
#endif /* DEFAULT_STATIC_ROBOT_SHAPE_H_ */
