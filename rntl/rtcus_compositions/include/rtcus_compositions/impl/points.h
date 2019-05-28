/*
 * points.h
 *
 *  Created on: Oct 25, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *      License: GPLv3
 */

#ifndef POINTS_H_
#define POINTS_H_

#include <mrpt/poses.h>
namespace rtcus_compositions
{
template<typename t_float>
  inline void inverse_compose_point_2D(t_float state_x, t_float state_y, t_float new_local_frame_x,
                                       t_float new_local_frame_y, t_float new_local_frame_phi, t_float&result_x,
                                       t_float&result_y)

  {
    mrpt::poses::CPoint2D state(state_x, state_y);
    mrpt::poses::CPose2D new_local_frame(new_local_frame_x, new_local_frame_y, new_local_frame_phi);
    mrpt::poses::CPoint2D result = state - new_local_frame;

    result_x = result.x();
    result_y = result.y();
  }

template<typename t_float>
  inline void compose_point_2d(const t_float&state_x, const t_float&state_y, const t_float&transform_x,
                               const t_float&transform_y, const t_float&transform_phi, t_float&result_x,
                               t_float&result_y)
  {

    double sx(state_x), sy(state_y), rx, ry;
    mrpt::poses::CPose2D transform(transform_x, transform_y, transform_phi);
    transform.composePoint(sx, sy, rx, ry);
    result_x = rx;
    result_y = ry;
  }

}
;

#endif /* POINTS_H_ */
