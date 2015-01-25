///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2014, David Conner, TORC Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of TORC Robotics, Team ViGIR, nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#ifndef HARDWARE_INTERFACE_VIGIR_HUMANOID_CONTROLLER_INTERFACE_H
#define HARDWARE_INTERFACE_VIGIR_HUMANOID_CONTROLLER_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>
#include <vigir_robot_model/VigirRobotModel.h>

namespace hardware_interface
{

/** A handle used to provide access to the controllable interfaces of the VigirHumanoidController. */
class VigirHumanoidControllerHandle
{
public:

  /**
   * \param name The name of the controller
   */
  VigirHumanoidControllerHandle() : name_(),
      joint_state_positions_      ( NULL )    ,
      joint_state_velocities_     ( NULL )   ,
      joint_state_accelerations_  ( NULL ),
      joint_state_efforts_        ( NULL )      ,
      joint_command_positions_    ( NULL ),
      joint_command_velocities_   ( NULL ),
      joint_command_accelerations_( NULL ),
      joint_command_efforts_      ( NULL ),
      joint_command_control_      ( NULL ),
      joint_command_friction_compensation_( NULL ),
      joint_position_errors_      ( NULL ),
      joint_velocity_errors_      ( NULL ),
      joint_effort_errors_        ( NULL ),
      current_bdi_control_gains_  ( NULL ),
      current_vigir_control_gains_( NULL ),
      robot_model_(NULL)
 {}

  VigirHumanoidControllerHandle(const std::string& name, const VectorNd *  joint_state_positions,
                                                 const VectorNd *  joint_state_velocities,
                                                 const VectorNd *  joint_state_accelerations,
                                                 const VectorNd *  joint_state_efforts,
                                                 VectorNd * joint_command_positions,             //!< desired position
                                                 VectorNd * joint_command_velocities,            //!< desired velocity
                                                 VectorNd * joint_command_accelerations,         //!< desired acceleration
                                                 VectorNd * joint_command_efforts,               //!< desired effort
                                                 VectorNd * joint_command_control,               //!< desired control command (acceleration)
                                                 VectorNd * joint_command_friction_compensation, //!< effort to compensate for friction
                                                 VectorNd * joint_position_errors,
                                                 VectorNd * joint_velocity_errors,
                                                 VectorNd * joint_effort_errors,
                                                 boost::shared_ptr<vigir_control::VigirRobotModel> robot_model)
      : name_(name),
        joint_state_positions_      (joint_state_positions)    ,
        joint_state_velocities_     (joint_state_velocities)   ,
        joint_state_accelerations_  (joint_state_accelerations),
        joint_state_efforts_        (joint_state_efforts)      ,
        joint_command_positions_    (joint_command_positions   ),
        joint_command_velocities_   (joint_command_velocities  ),
        joint_command_accelerations_(joint_command_accelerations),
        joint_command_efforts_      (joint_command_efforts     ),
        joint_command_control_      (joint_command_control     ),
        joint_command_friction_compensation_(joint_command_friction_compensation),
        joint_position_errors_      (joint_position_errors     ),
        joint_velocity_errors_      (joint_velocity_errors     ),
        joint_effort_errors_        (joint_effort_errors       ),
        robot_model_(robot_model)
  {
      if (NULL == joint_state_positions_       ) throw HardwareInterfaceException("Cannot create handle '" + name + "' some pointer is null.");
      if (NULL == joint_state_velocities_      ) throw HardwareInterfaceException("Cannot create handle '" + name + "' some pointer is null.");
      if (NULL == joint_state_accelerations_   ) throw HardwareInterfaceException("Cannot create handle '" + name + "' some pointer is null.");
      if (NULL == joint_state_efforts_         ) throw HardwareInterfaceException("Cannot create handle '" + name + "' some pointer is null.");
      if (NULL == joint_command_positions_     ) throw HardwareInterfaceException("Cannot create handle '" + name + "' some pointer is null.");
      if (NULL == joint_command_velocities_    ) throw HardwareInterfaceException("Cannot create handle '" + name + "' some pointer is null.");
      if (NULL == joint_command_accelerations_ ) throw HardwareInterfaceException("Cannot create handle '" + name + "' some pointer is null.");
      if (NULL == joint_command_efforts_       ) throw HardwareInterfaceException("Cannot create handle '" + name + "' some pointer is null.");
      if (NULL == joint_command_control_       ) throw HardwareInterfaceException("Cannot create handle '" + name + "' some pointer is null.");
      if (NULL == joint_command_friction_compensation_ ) throw HardwareInterfaceException("Cannot create handle '" + name + "' some pointer is null.");
      if (NULL == joint_position_errors_       ) throw HardwareInterfaceException("Cannot create handle '" + name + "' some pointer is null.");
      if (NULL == joint_velocity_errors_       ) throw HardwareInterfaceException("Cannot create handle '" + name + "' some pointer is null.");
      if (NULL == joint_effort_errors_         ) throw HardwareInterfaceException("Cannot create handle '" + name + "' some pointer is null.");
      if (NULL == robot_model_.get()           ) throw HardwareInterfaceException("Cannot create handle '" + name + "' some pointer is null.");

  }

  std::string getName() const {return name_;}

  // Keep these public for access by controller (yes - this is bad form)
  const VectorNd * joint_state_positions_;
  const VectorNd * joint_state_velocities_;
  const VectorNd * joint_state_accelerations_;
  const VectorNd * joint_state_efforts_;
  VectorNd *  joint_command_positions_;             //!< desired position
  VectorNd *  joint_command_velocities_;            //!< desired velocity
  VectorNd *  joint_command_accelerations_;         //!< desired acceleration
  VectorNd *  joint_command_efforts_;               //!< desired effort
  VectorNd *  joint_command_control_;               //!< desired control command (acceleration)
  VectorNd *  joint_command_friction_compensation_; //!< effort to compensate for friction
  VectorNd *  joint_position_errors_;
  VectorNd *  joint_velocity_errors_;
  VectorNd *  joint_effort_errors_;
  boost::shared_ptr<vigir_control::VigirRobotModel> robot_model_;

private:
  std::string name_;

};

/** \brief Hardware interface to support access of controller data by whole body controllers
 *
 */
class VigirHumanoidControllerInterface : public HardwareResourceManager<VigirHumanoidControllerHandle, ClaimResources> {};

}

#endif
