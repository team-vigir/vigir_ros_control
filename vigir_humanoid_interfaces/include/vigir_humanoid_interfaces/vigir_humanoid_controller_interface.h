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
      robot_pose_(NULL)
 {}

  VigirHumanoidControllerHandle(const std::string& name, const vigir_control::VectorNd *  joint_state_positions,
                                                 const vigir_control::VectorNd *  joint_state_velocities,
                                                 const vigir_control::VectorNd *  joint_state_accelerations,
                                                 const vigir_control::VectorNd *  joint_state_efforts,
                                                 vigir_control::VectorNd * joint_command_positions,             //!< desired position
                                                 vigir_control::VectorNd * joint_command_velocities,            //!< desired velocity
                                                 vigir_control::VectorNd * joint_command_accelerations,         //!< desired acceleration
                                                 vigir_control::VectorNd * joint_command_efforts,               //!< desired effort
                                                 vigir_control::VectorNd * joint_command_control,               //!< desired control command (acceleration)
                                                 vigir_control::VectorNd * joint_command_friction_compensation, //!< effort to compensate for friction
                                                 vigir_control::VectorNd * joint_position_errors,
                                                 vigir_control::VectorNd * joint_velocity_errors,
                                                 vigir_control::VectorNd * joint_effort_errors,
                                                 vigir_control::Pose     * robot_pose,
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
        robot_pose_(robot_pose),
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
      if (NULL == robot_pose_                  ) throw HardwareInterfaceException("Cannot create handle '" + name + "' robot pose pointer is null.");
      if (NULL == robot_model_.get()           ) throw HardwareInterfaceException("Cannot create handle '" + name + "' some pointer is null.");

  }

  std::string getName() const {return name_;}

  // Keep these public for access by controller (yes - this is bad form)
  const vigir_control::VectorNd * joint_state_positions_;
  const vigir_control::VectorNd * joint_state_velocities_;
  const vigir_control::VectorNd * joint_state_accelerations_;
  const vigir_control::VectorNd * joint_state_efforts_;
  vigir_control::VectorNd *  joint_command_positions_;             //!< desired position
  vigir_control::VectorNd *  joint_command_velocities_;            //!< desired velocity
  vigir_control::VectorNd *  joint_command_accelerations_;         //!< desired acceleration
  vigir_control::VectorNd *  joint_command_efforts_;               //!< desired effort
  vigir_control::VectorNd *  joint_command_control_;               //!< desired control command (acceleration)
  vigir_control::VectorNd *  joint_command_friction_compensation_; //!< effort to compensate for friction
  vigir_control::VectorNd *  joint_position_errors_;
  vigir_control::VectorNd *  joint_velocity_errors_;
  vigir_control::VectorNd *  joint_effort_errors_;
  vigir_control::Pose     *  robot_pose_;
  boost::shared_ptr<vigir_control::VigirRobotModel> robot_model_;

protected:
  std::string name_;

};

/** \brief Hardware interface to support access of controller data by whole body controllers
 *  NOTE: This interface DOES NOT claim handle resources, and therefore it is possible that multiple controllers could
 * interfere with one another.  It is up to the user to define what controllers are active and ensure that they cooperate
 * appropriately.
 */
class VigirHumanoidControllerInterface : public HardwareResourceManager<VigirHumanoidControllerHandle, DontClaimResources> {};

}

#endif
