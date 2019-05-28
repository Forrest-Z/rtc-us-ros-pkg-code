/*
 * deterministic_non_holonomic_motion_model.h
 *
 *  Created on: Apr 10, 2012
 *      Author: Pablo Iñigo Blasco - Robotics and Computer Architecture (ATC) - University of Seville - 2012
 *
 */

#ifndef DETERMINISTIC_NON_HOLONOMIC_ICR_2D_H
#define DETERMINISTIC_NON_HOLONOMIC_ICR_2D_H

#include <rtcus_motion_models/motion_models.h>
#include <rtcus_motion_models/motion_models/motion_models_impl.h>

namespace rtcus_motion_models
{

/*!
 * @brief this function represent the local kinematic 2D non-holonomic state transition independent of specific data types
 * */
void predictLocalStateTransition(double v, double omega, double dtt_secs, double &final_x, double& final_y,
                                 double& final_phi);
void predictLocalStateTransition(double v, double omega, double dtt_secs, float &final_x, float& final_y,
                                 float& final_phi);

template<typename StateType, typename ActionType>
  class DeterministicNonHolonomic2D : public KinematicMotionModel<StateType, ActionType>
  {
  private:
    typedef double t_float;
  public:
    DeterministicNonHolonomic2D() :
        KinematicMotionModel<StateType, ActionType>::KinematicMotionModel()
    {
    }

    virtual ~DeterministicNonHolonomic2D()
    {
    }

    virtual void predictLocalStateTransition(const ActionType& action, const ros::Duration dt,
                                             StateType& final_state) const;

  };

}

#endif /* DETERMINISTIC_NON_HOLONOMIC_ICR_2D_H */
