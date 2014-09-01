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
#ifndef __VIGIR_HUMANOID_STATUS_CODES_H__
#define __VIGIR_HUMANOID_STATUS_CODES_H__

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>

//@todo RobotMode (startup) , RobotBehavior, and RobotFootsteps interfaces

#include <vigir_humanoid_controller/VigirHumanoidInterface.h>


namespace vigir_control {


    enum VigirHumanoidInitializationStatus{
        ROBOT_EXCEPTION_SUBSCRIBERS_FAILED_TO_INITIALIZE = -6,
        ROBOT_EXCEPTION_MODEL_FAILED_TO_INITIALIZE       = -5,
        ROBOT_EXCEPTION_INTERFACE_FAILED_TO_INITIALIZE   = -4,
        ROBOT_EXCEPTION_BEHAVIORS_FAILED_TO_INITIALIZE   = -3,
        ROBOT_EXCEPTION_CONTROLLERS_FAILED_TO_INITIALIZE = -2,
        ROBOT_EXCEPTION_PUBLISHERS_FAILED_TO_INITIALIZE  = -1,
        ROBOT_INITIALIZED_OK = 0,
        ROBOT_MODEL_NOT_DEFINED,
        ROBOT_MODEL_NO_ROBOT_DESCRIPTION,
        ROBOT_MODEL_ROBOT_DESCRIPTION_FAILED_TO_LOAD,
        ROBOT_MODEL_NO_CONTROLLED_JOINTS_LIST,
        ROBOT_MODEL_CONTROLLED_JOINTS_LIST_FAILED_TO_LOAD,
        ROBOT_MODEL_CONTROLLED_JOINTS_FAILED_TO_INITIALIZE,
        ROBOT_MODEL_FAILED_TO_INITIALIZE,
        ROBOT_INTERFACE_FAILED_TO_INITIALIZE,
        ROBOT_BEHAVIORS_FAILED_TO_INITIALIZE,
        ROBOT_CONTROLLERS_FAILED_TO_INITIALIZE,
        ROBOT_SUBSCRIBERS_FAILED_TO_INITIALIZE,
        ROBOT_PUBLISHERS_FAILED_TO_INITIALIZE
    };
    enum VigirHumanoidCleanupStatus{
        ROBOT_EXCEPTION_SUBSCRIBERS_FAILED_TO_CLEANUP_PROPERLY  = -6,
        ROBOT_EXCEPTION_MODEL_FAILED_TO_CLEANUP_PROPERLY        = -5,
        ROBOT_EXCEPTION_INTERFACE_FAILED_TO_CLEANUP_PROPERLY    = -4,
        ROBOT_EXCEPTION_BEHAVIORS_FAILED_TO_CLEANUP_PROPERLY    = -3,
        ROBOT_EXCEPTION_CONTROLLERS_FAILED_TO_CLEANUP_PROPERLY  = -2,
        ROBOT_EXCEPTION_PUBLISHERS_FAILED_TO_CLEANUP_PROPERLY   = -1,
        ROBOT_CLEANUP_OK = 0,
        ROBOT_MODEL_FAILED_TO_CLEANUP_PROPERLY,
        ROBOT_INTERFACE_FAILED_TO_CLEANUP_PROPERLY   ,
        ROBOT_BEHAVIORS_FAILED_TO_CLEANUP_PROPERLY,
        ROBOT_CONTROLLERS_FAILED_TO_CLEANUP_PROPERLY,
        ROBOT_SUBSCRIBERS_FAILED_TO_CLEANUP_PROPERLY,
        ROBOT_PUBLISHERS_FAILED_TO_CLEANUP_PROPERLY
    };



} // end of vigir_control namespace
#endif

