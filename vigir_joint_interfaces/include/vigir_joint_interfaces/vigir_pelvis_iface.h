/*=================================================================================================
// Copyright (c) 2013-2014, David Conner, TORC Robotics
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Team ViGIR or TORC Robotics nor the names of
//       its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================
*/


#ifndef VIGIR_PELVIS_CMD_INTERFACE_H
#define VIGIR_PELVIS_CMD_INTERFACE_H

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_command_interface.h>

namespace hardware_interface
{


/** \brief A handle used to read and command a single joint. */
class VigirPelvisHandle : public JointHandle
{
public:
  VigirPelvisHandle() : hardware_interface::JointHandle(), use_desired_(0) {}

  /**
   * \param js This joint's handle
   * \param desired_pos A pointer to the storage for this pelvis axis's position command
   * \param use_desired A pointer to the storage for boolean flag denoting we are ready to use pelvis commands
   */
  VigirPelvisHandle(const JointStateHandle& js, double* desired_pos,double * posn_error, bool  * use_desired)
    : JointHandle(js, desired_pos),  desired_pos_(desired_pos), posn_error_(posn_error), use_desired_(use_desired)
  {
      if (!desired_pos_ || !use_desired_ || !posn_error_)
      {
        throw HardwareInterfaceException(
            "Cannot create handle '" + js.getName() + "'. Some command data pointer is null.");
      }
  }

  // JointHandle interface used cmd_ as the desired position (this accesses the same pointer)
  inline void   setPositionCommand(double pos) {assert(desired_pos_); *desired_pos_ = pos;}
  inline double getPositionCommand() const {assert(desired_pos_); return *desired_pos_;}

  inline bool   getUseDesiredPelvisCommand() {assert(use_desired_); return *use_desired_ ;}


private:

  double* desired_pos_;
  double* posn_error_;
  bool  * use_desired_;
};

/** \brief Hardware interface to support commanding an array of pelvis commands by position.
 *
 * \note Getting a joint handle through the getHandle() method \e will claim that resource.
 *
 */
class VigirPelvisInterface :
  public HardwareResourceManager<VigirPelvisHandle, ClaimResources> {};

}

#endif
