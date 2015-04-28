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


#ifndef VIGIR_PELVIS_CMD_INTERFACE_ADAPTER_H
#define VIGIR_PELVIS_CMD_INTERFACE_ADAPTER_H

#include <joint_trajectory_controller/hardware_interface_adapter.h>
#include <vigir_humanoid_interfaces/vigir_pelvis_iface.h>


/**
 * \brief Adapter for a position-controlled hardware interface.
 * Forwards desired positions as commands if the use_desired flag is set.
 */
template <class State>
class HardwareInterfaceAdapter<hardware_interface::VigirPelvisInterface, State> :
        public HardwareInterfaceAdapter<hardware_interface::PositionJointInterface, State>
{
public:
    HardwareInterfaceAdapter() : pelvis_handles_ptr_(0), n_joints_(0) {}

  bool init(std::vector<hardware_interface::VigirPelvisHandle>& pelvis_handles,
                        ros::NodeHandle& controller_nh)
  {

    //ROS_INFO(" Inside HardwareInterfaceAdapter<hardware_interface::VigirVigirJointInterface, State>");

    // Store pointer to joint handles
    pelvis_handles_ptr_ = &pelvis_handles;
    n_joints_ = pelvis_handles.size();

    return true;
  }

  void starting(const ros::Time& time)
  {
      ROS_INFO_THROTTLE(0.5,"    STARTING Vigir pelvis trajectory controller - update controller gains");
  }

  void stopping(const ros::Time& time)
  {
      ROS_INFO_THROTTLE(0.5,"    STOPPING Vigir pelvis trajectory controller ");
  }

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& period,
                     const State&         desired_state,
                     const State&         state_error)
  {
    if ((*pelvis_handles_ptr_)[0].getInPelvisControlMode())
    {
      (*pelvis_handles_ptr_)[0].setUseDesiredPelvisCommand(1); // flag to use this value
      for (unsigned int i = 0; i < n_joints_; ++i) {
        // Update the desired commands based on the desired trajectory
        (*pelvis_handles_ptr_)[i].setPositionCommand(desired_state.position[i]);
      }
    }
    else
    {
      for (unsigned int i = 0; i < n_joints_; ++i) {
        // Track what is fed back if we are not using the desired pelvis data
        (*pelvis_handles_ptr_)[i].setPositionCommand((*pelvis_handles_ptr_)[i].getPosition());
      }
    }
  }

  bool getInPelvisControlMode() {if (pelvis_handles_ptr_) return (*pelvis_handles_ptr_)[0].getInPelvisControlMode(); else return false;};

private:

  uint32_t                                                                          n_joints_;
  std::vector<hardware_interface::VigirPelvisHandle>*                               pelvis_handles_ptr_;

};

#endif
