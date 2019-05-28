/*
 * input_sources.h
 *
 *  Created on: Mar 28, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef WORLD_PERCEPTION_H_
#define WORLD_PERCEPTION_H_

#include <rtcus_navigation/core.h>
#include <boost/thread.hpp>

//this is one of the most important components. Performance and avoid data duplication
//is one of the targets

//from now input sources are supposed to be given in the local reference frame
//multiple input sources are not still allowed but could be multiplexes by this mechanism
//why depends on the state? the current state can affect to the perception of the system..

//given a prediction
//the typically way to implement this method is transform all the obstacle pose information to the
//new estimated local frame and also consider the uncertainty on them
namespace rtcus_navigation
{

using namespace rtcus_stamp;
using namespace boost;

/**
 * \brief This class represent the input of the perceived obstacles in the world. It also can
 * enrich the perceived reading before being given to the navigation node.
 * \remarks it is recommended to give the obstacles in the local frame since most of planner are ready
 * for work which such information.
 * */
template<typename ObstaclesType, typename TimeModel>
  class WorldPerceptionPort : public NavigationNodeComponent
  {
    USING_TIME_MODEL(TimeModel);

  public:
    virtual ~WorldPerceptionPort()
    {
    }
    ;

    virtual void reset()=0;
    virtual void init()=0;

    /**
     * \brief gets the world perceived at the last obstacle reading. It is o(t_o)
     * \return optionally a mutex which must be given already locked. If a nullptr is given the last update world is not
     * thread safe or it is expected to be given as a copy.
     *
     * \remarks If an StampedData<ObstacleType> is given, the obstacle data is owned by the external user. This is useful
     * when the command allocateObstacleRepresentation has been used before externally.
     */
    virtual shared_ptr<mutex> getLastUpdateWorld(Stamped<ObstaclesType, TTime>& obstacles) =0;

    /* \brief gets the world perceived at the last obstacle reading. It is o(t_o)
     * \return optionally a mutex which must be given already locked. If a nullptr is given the last update world is not
     * thread safe or it is expected to be given as a copy.
     *
     * \remarks If an StampedData<ObstacleType> is given, the obstacle data is owned by the external user. This is useful
     * when the command allocateObstacleRepresentation has been used before externally.
     */
    virtual shared_ptr<mutex> getWorldEstimation(TTime time, Stamped<ObstaclesType, TTime> & obstacles) =0;

    /*\brief while this function return false the planning process is stepped.
     * */
    virtual bool hasValidObstaclesEstimation()=0;

    virtual void pushObstacles(const ObstaclesType& obstacles)=0;

    boost::signal<void(WorldPerceptionPort<ObstaclesType, TimeModel>& sender)> onWorldUpdate;

    virtual void setSensorFrameName(const std::string&)=0;
    virtual std::string getSensorFrameName() const=0;
    virtual void setMobileBaseFrameName(const std::string&)=0;
    virtual std::string getMobileBaseFrameName() const=0;
    virtual void allocateObstacleRepresentation(Stamped<ObstaclesType, TTime>& obstacles)=0;

  protected:
    WorldPerceptionPort() :
        NavigationNodeComponent(tPerceptionPort)
    {
    }
  };
}
#endif /* INPUT_SOURCES_H_ */
