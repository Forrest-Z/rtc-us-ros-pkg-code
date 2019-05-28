/*
 * poses.h
 *
 *  Created on: Oct 25, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */
#ifndef POSES_H_
#define POSES_H_

#include <mrpt/poses.h>
namespace rtcus_compositions
{
template<typename t_float>
  inline void inverse_compose_pose_2d(const t_float&state_x, const t_float&state_y, const t_float&state_phi,
                                      const t_float&new_local_frame_x, const t_float&new_local_frame_y,
                                      const t_float&new_local_frame_phi, t_float&result_x, t_float&result_y,
                                      t_float&result_phi)
  {
    mrpt::poses::CPose2D state(state_x, state_y, state_phi);
    mrpt::poses::CPose2D new_local_frame(new_local_frame_x, new_local_frame_y, new_local_frame_phi);
    mrpt::poses::CPose2D result = state - new_local_frame;

    result_x = result.x();
    result_y = result.y();
    result_phi = result.phi();
  }

template<typename t_float>
  inline void compose_pose_2d(const t_float&state_x, const t_float&state_y, const t_float&state_phi,
                              const t_float&transform_x, const t_float&transform_y, const t_float&transform_phi,
                              t_float&result_x, t_float&result_y, t_float&result_phi)
  {
    mrpt::poses::CPose2D state(state_x, state_y, state_phi);
    mrpt::poses::CPose2D transform(transform_x, transform_y, transform_phi);
    mrpt::poses::CPose2D result = state + transform;

    result_x = result.x();
    result_y = result.y();
    result_phi = result.phi();
  }
}
;
#endif

