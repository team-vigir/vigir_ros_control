/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//@TODO_ADD_AUTHOR_INFO

// taken from https://github.com/gt-ros-pkg/universal_robot/tree/hydro-devel-c-api/ur_ctrl_client
/// \author Kelsey Hawkins

#ifndef POS_VEL_ACC_JOINT_CMD_INTERFACE_H
#define POS_VEL_ACC_JOINT_CMD_INTERFACE_H

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_command_interface.h>

namespace hardware_interface
{

/** \brief A handle used to read and command a single joint. */
class PosVelAccJointHandle : public JointHandle
{
public:
  PosVelAccJointHandle() : hardware_interface::JointHandle(), vel_(0), acc_(0) {}

  /**
   * \param js This joint's handle
   * \param pos A pointer to the storage for this joint's position command
   * \param vel A pointer to the storage for this joint's velocity command
   * \param acc A pointer to the storage for this joint's acceleration command
   */
  PosVelAccJointHandle(const JointStateHandle& js, double* pos, double* vel, double* acc)
    : JointHandle(js, pos), vel_(vel), acc_(acc)
  {
    if (!vel_ || !acc_)
    {
      throw HardwareInterfaceException(
          "Cannot create handle '" + js.getName() + "'. Some command data pointer is null.");
    }
  }

  void   setPositionCommand(double pos) {setCommand(pos);}
  double getPositionCommand() const {return getCommand();}

  void   setVelocityCommand(double vel) {assert(vel_); *vel_ = vel;}
  double getVelocityCommand() const {assert(vel_); return *vel_;}

  void   setAccelerationCommand(double acc) {assert(acc_); *acc_ = acc;}
  double getAccelerationCommand() const {assert(acc_); return *acc_;}

private:
  double* vel_;
  double* acc_;
};

/** \brief Hardware interface to support commanding an array of joints by position velocity and acceleration.
 *
 * \note Getting a joint handle through the getHandle() method \e will claim that resource.
 *
 */
class PosVelAccJointInterface : 
  public HardwareResourceManager<PosVelAccJointHandle, ClaimResources> {};

}

#endif
