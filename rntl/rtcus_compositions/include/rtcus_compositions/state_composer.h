/*
 * state_composer.h
 *
 *  Created on: Jun 1, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef STATE_COMPOSER_H_
#define STATE_COMPOSER_H_

#include <string>

namespace rtcus_compositions
{

#define ANONYMOUS_LOCAL_FRAME_NAME "anonymous_local_frame_name"

//TODO: Rename this to SE_n_GeometricComposer
class StateComposer
{
public:

  template<typename targetType, typename localStateFrame>
    static void compose(const targetType& src, const localStateFrame& delta_transform, targetType& dst);

  template<typename t_target, typename t_local_frame>
    static void inverse_compose(const t_target& initial_state, const t_local_frame& local_frame_state, t_target& dst,
                                const std::string& new_frame_name = ANONYMOUS_LOCAL_FRAME_NAME);

  //TODO: use the inverse compose notation with frame id with Stamped<T> data, and forget it for the implementation
  //or with objects which have header through template spetialization.

};

}

#endif /* STATE_COMPOSER_H_ */
