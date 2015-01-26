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

#ifndef ATLAS_GRAVITY_COMPENSATION_CONTROLLER_CONTROLLER___
#define ATLAS_GRAVITY_COMPENSATION_CONTROLLER_CONTROLLER___

#include <controller_interface/controller.h>

#include <vigir_humanoid_controllers/vigir_controller_controller_base.h>

#include <actionlib/server/action_server.h>

//// actionlib
//#include <realtime_tools/realtime_box.h>
//#include <realtime_tools/realtime_server_goal_handle.h>

// This controller calculates the required effort to compensate for gravity at the instantaneous desired position
namespace vigir_humanoid_controllers
{

  class VigirGravityCompensationController : public VigirControllerControllerBase
  {
    public:
      bool init(hardware_interface::VigirControllerInterface* hw, ros::NodeHandle &nh);

      void update(const ros::Time& time, const ros::Duration& period);

      void starting(const ros::Time& time);
      void stopping(const ros::Time& time);

    private:

      hardware_interface::VigirControllerHandle controller_handle_;
      VectorNd      joint_efforts_;
  };

}

#endif
