/*
 *
 *  Created on: Apr 6, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef STATE_PORT_H_
#define STATE_PORT_H_

#include <rtcus_navigation/core.h>

namespace rtcus_navigation
{
template<typename StateType, typename TimeModel>
  /**
   * \brief This core class interface takes the current state reading from external sensors.
   *
   *  TODO: This class interface is expected to be merged with the state estimation core class interface. This is to follow
   *  the same approach followed by the WorldPerceptionPort component
   */
  class StatePort : public NavigationNodeComponent
  {
    USING_TIME_MODEL(TimeModel);

  public:
    /*
     * \brief The state port implementation decides how to allocate the state data. The specific state port
     * could decide to give an internal reference, or even to allocate in the heap for you if your
     * reference does not have allocated data.
     *  - To get a copy: Pass an StampedData<StateType> object
     *  - To get a reference (if port implementation agrees): Pass a heap allocated Stamped<StateType> object (no StampedData)
     *  - To allow the port allocate the data for you or pass a reference: Pass an unallocated StampedData<StateType> variable. This method
     *  is used by the NavigationNode implementation to allow as many port types as possible
     *  Pay attention because a bad use of this function can be inefficent if the obstacle output parameter is a local scope variable.
     *
     * @remarks: In any case it is recommended to pass as argument an StampedData object which contains static allocation.
     * In this case the port will pass a copy, since this type does not allow referenced data (like the Stamped<StateType> type)
     * */
    virtual void getLastState(rtcus_stamp::Stamped<StateType, TTime>& last_state) =0;

    virtual void copyLastState(rtcus_stamp::StampedData<StateType, TTime>& last_state) const=0;

    virtual bool hasValidState() const=0;

    /*! @brief insert by software a state reading. The state should be stamped in time and
     * frame id.
     * */
    virtual void pushState(const rtcus_stamp::StampedData<StateType, TTime>& new_state)=0;
    boost::signal<void(const StatePort<StateType, TimeModel>& sender, const TTime& reception_time)> onStateReceived;

    virtual void reset()=0;
    virtual void init(std::string reference_frame)=0;

    virtual ~StatePort()
    {
    }
    ;
  protected:
    StatePort() :
        NavigationNodeComponent(tStatePort)
    {
    }
  };
}
#endif /* STATE_PORT_H_ */
