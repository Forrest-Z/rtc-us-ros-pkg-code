/*
 * custom_configurable_static_robot_shape.h
 *
 *  Created on: Dec 9, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef CUSTOM_CONFIGURABLE_STATIC_ROBOT_SHAPE_H_
#define CUSTOM_CONFIGURABLE_STATIC_ROBOT_SHAPE_H_

#include <rtcus_navigation/core.h>
#include <rtcus_navigation/shape_models/default_static_robot_shape.h>
#include <dynamic_reconfigure/server.h>
#include <boost/type_traits.hpp>
#include <rtcus_assert/rtcus_assert.h>
#include <boost/make_shared.hpp>

namespace rtcus_navigation
{
namespace shape_models
{

/**
 * \brief this class wraps another RobotShapeModel and exposes its data on a dynamic_reconfigure server (assuming than the type RosShapeType is a compatible type with it)
 */

template<typename RobotShapeType, typename TimeModel = ROSTimeModel>
  class ConfigurableRobotShapeModel : public RobotShapeModel<RobotShapeType, TimeModel>
  {
    USING_TIME_MODEL (TimeModel);

  private:
    boost::shared_ptr<RobotShapeModel<RobotShapeType, TimeModel> > decorated_;
    boost::shared_ptr<dynamic_reconfigure::Server<RobotShapeType> > dynamic_reconfigure_server_;

  protected:
    virtual RobotShapeType onShapeConfigurationUpdated(RobotShapeType &config, uint32_t level)
    {
      this->setRobotShape(config);
      this->getRobotShape(config);
      return config;
    }

    virtual void onDecoratedRobotShapeChanged(const RobotShapeType& shape)
    {
      //bypass the decorated event
      this->onShapeChanged(shape);
    }

  public:
    virtual ~ConfigurableRobotShapeModel()
    {
    }

    ConfigurableRobotShapeModel()

    {
      this->decorated_ = boost::make_shared<DefaultStaticRobotShape<RobotShapeType, TimeModel> >();
    }
    ConfigurableRobotShapeModel(boost::shared_ptr<RobotShapeModel<RobotShapeType, TimeModel> > decorated)

    {
      RTCUS_ASSERT(decorated);
      this->decorated_ = decorated;
    }

    /*
     void printfParameterServer(std::string at)
     {
     XmlRpc::XmlRpcValue current_config_str;
     this->component_node_.getParam((this->getComponentNode().getNamespace()).c_str(), current_config_str);

     std::string result = current_config_str.toXml();
     ROS_INFO_STREAM(
     "["<<at<<" PARAMETER SERVER] Current shape info ["<<this->component_node_.getNamespace().c_str()<<"]:"<< result);

     }*/

    virtual void init()
    {
      decorated_->init();

      this->dynamic_reconfigure_server_ = boost::shared_ptr<dynamic_reconfigure::Server<RobotShapeType> >(
          new dynamic_reconfigure::Server<RobotShapeType>(this->component_node_));

      typedef typename dynamic_reconfigure::Server<RobotShapeType>::CallbackType configure_server_callback;
      configure_server_callback f = boost::bind(
          &ConfigurableRobotShapeModel<RobotShapeType, TimeModel>::onShapeConfigurationUpdated, this, _1, _2);
      dynamic_reconfigure_server_->setCallback(f);

      RobotShapeType current_shape;
      decorated_->getRobotShape(current_shape);

      decorated_->onShapeChanged.connect(
          boost::bind(&ConfigurableRobotShapeModel<RobotShapeType, TimeModel>::onDecoratedRobotShapeChanged, this, _1));

    }

    virtual void reset()
    {
      decorated_->reset();
    }

    virtual void getRobotShape(RobotShapeType& shape) const
    {
      decorated_->getRobotShape(shape);
    }

    virtual void setRobotShape(const RobotShapeType& shape)
    {
      decorated_->setRobotShape(shape);
    }

    virtual void predictRobotShape(TTime time, RobotShapeType& shape)
    {
      decorated_->predictRobotShape(time, shape);
    }
  };

}
}

#endif /* CUSTOM_CONFIGURABLE_STATIC_ROBOT_SHAPE_H_ */
