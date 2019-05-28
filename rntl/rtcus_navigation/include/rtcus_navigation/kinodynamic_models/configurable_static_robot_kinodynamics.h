/*
 * circular_robot_shape.h
 *
 *  Created on: Dec 8, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef ROBOT_KINODYNAMIC_CONFIGURABLE_H_
#define ROBOT_KINODYNAMIC_CONFIGURABLE_H_

#include <rtcus_navigation/kinodynamic_models/default_static_robot_kinodynamics.h>
#include <dynamic_reconfigure/server.h>
#include <boost/type_traits.hpp>

namespace rtcus_navigation
{
namespace kinodynamic_models
{

using namespace boost;

/**
 * \brief this is an utility class which allow to use KinoDynamicDescriptions generated automatically by the dynamic reconfigure server.
 * It uses a decorator pattern and allows to extend an existing KinoDynamicModel based on this description.
 * */
template<typename KinoDynamicDescription>
  class ConfigurableKinodynamicModel : public KinoDynamicModel<KinoDynamicDescription>
  {
  private:
    shared_ptr<KinoDynamicModel<KinoDynamicDescription> > decorated_;
    dynamic_reconfigure::Server<KinoDynamicDescription> dynamic_reconfigure_server_;

  protected:
    virtual KinoDynamicDescription onKinoDynamicConfigurationUpdated(KinoDynamicDescription &config, uint32_t level)
    {
      //stores the configuration
      decorated_->setKinoDynamics(config);

      //get the accepted values
      decorated_->getKinodynamics(config);

      //returning it to the configuration server
      return config;
    }

    virtual void onKinoDynamicChanged(const KinoDynamicDescription& kd)
    {
      //bypass the decorated event
      this->onKinodynamicsChanged(kd);
    }

  public:

    ConfigurableKinodynamicModel() :
        dynamic_reconfigure_server_(this->component_node_)
    {
      this->decorated_ = boost::make_shared<DefaultKinodynamicModel<KinoDynamicDescription> >();

    }
    ConfigurableKinodynamicModel(shared_ptr<KinoDynamicModel<KinoDynamicDescription> > kdmodel) :
        dynamic_reconfigure_server_(this->component_node_)
    {
      this->decorated_ = kdmodel;
    }

    /**
     * \brief this constructor allows to build the component in a non default place in the navigation architecture.
     * */
    ConfigurableKinodynamicModel(shared_ptr<KinoDynamicModel<KinoDynamicDescription> > kdmodel, ros::NodeHandle nh) :
        dynamic_reconfigure_server_(nh)
    {
      this->decorated_ = kdmodel;
    }

    virtual ~ConfigurableKinodynamicModel()
    {
    }

    virtual void init()
    {
      decorated_->init();

      typedef typename dynamic_reconfigure::Server<KinoDynamicDescription>::CallbackType configure_server_callback;
      configure_server_callback f = boost::bind(
          &ConfigurableKinodynamicModel<KinoDynamicDescription>::onKinoDynamicConfigurationUpdated, this, _1, _2);
      dynamic_reconfigure_server_.setCallback(f);

      decorated_->onKinodynamicsChanged.connect(
          boost::bind(&ConfigurableKinodynamicModel<KinoDynamicDescription>::onKinoDynamicChanged, this, _1));

      //because the configure server may have changed them
      KinoDynamicDescription kd;
      this->getKinodynamics(kd);
      this->onKinodynamicsChanged(kd);
    }

    virtual void reset()
    {
      decorated_->reset();
    }

    virtual void getKinodynamics(KinoDynamicDescription& kinodynamics) const
    {
      decorated_->getKinodynamics(kinodynamics);
    }

    virtual void setKinoDynamics(const KinoDynamicDescription& knds)
    {
      decorated_->setKinoDynamics(knds);
    }

  };
}
}

#endif /* CIRCULAR_ROBOT_SHAPE_H_ */
