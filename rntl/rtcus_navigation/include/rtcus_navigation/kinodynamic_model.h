/*
 * robot_kinodynamics.h
 *
 *  Created on: Dec 8, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef ROBOT_KINODYNAMICS_H_
#define ROBOT_KINODYNAMICS_H_

#include <rtcus_navigation/core.h>

namespace rtcus_navigation
{
template<typename KinoDynamicDescription>
  class KinoDynamicModel : public NavigationNodeComponent
  {
  protected:
    KinoDynamicModel() :
        NavigationNodeComponent(tKinoDynamicModel)
    {

    }
  public:
    typedef KinoDynamicDescription TKinoDynamicDescription;
    virtual void reset()=0;
    virtual void init()=0;

    virtual void getKinodynamics(KinoDynamicDescription& kinodynamics) const=0;
    virtual void setKinoDynamics(const KinoDynamicDescription& knds)=0;

    boost::signal<void(const KinoDynamicDescription&)> onKinodynamicsChanged;
    virtual ~KinoDynamicModel()
    {
    }

  };
}

#endif /* ROBOT_KINODYNAMICS_H_ */
