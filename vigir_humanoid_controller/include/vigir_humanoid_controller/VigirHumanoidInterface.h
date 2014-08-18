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
#ifndef __VIGIR_HUMANOID_INTERFACE_H__
#define __VIGIR_HUMANOID_INTERFACE_H__

#include <string>
#include <boost/shared_ptr.hpp>

#include <vigir_robot_model/VigirRobotModel.h>
#include <vigir_robot_model/VigirRobotState.h>
#include <vigir_robot_model/VigirRobotFilterBase.h>
#include <vigir_robot_model/VigirRobotPoseFilterBase.h>

namespace vigir_control {

class VigirRobotBehaviorData; // temporary dummy class definition

/**
 * This structure defines the interface for a humanoid robot and
 * serves as the base for the actual robot interface.
 *
 * This structure (and its robot specific derivatives) provide the
 * interface to the robot and its data.
 *
 * This is NOT the RobotHW interface used by the controller.
 *    See VigirHumanoidController for the RobotHW interface.
 */
  struct VigirHumanoidInterface
  {

    VigirHumanoidInterface(const std::string& name, boost::shared_ptr<ros::NodeHandle>& pub_nh)
        : name_(name),pub_nh_(pub_nh) { }
    virtual ~VigirHumanoidInterface() {}


    // Implementation specific functions that provide access to robot
    // data.
    // NOTE: The robot specific implementation is responsible for
    //    providing data protection in multithreaded environments
    //    as these may be called at any time from the robot interface
    //    and accessed by the

    virtual int32_t initialize_models() = 0;
    virtual int32_t initialize_interface() = 0;
    virtual int32_t cleanup_models() = 0;
    virtual int32_t cleanup_interface() = 0;

    // These functions assume that data is protected in multithreaded environments before call
    virtual void update_state_data()    = 0; // from robot
    virtual void publish_state_data()   = 0; // to ROS
    virtual void send_controller_data() = 0; // to robot

    virtual void update_behavior_data()  = 0; // from robot
    virtual void publish_behavior_data() = 0; // to ROS
    virtual void send_behavior_data()    = 0; // to robot

    std::string                           name_;

    boost::shared_ptr<ros::NodeHandle>    pub_nh_;        // Node handle for common data publishing

    // The following classes are defined by the implementation
    boost::shared_ptr<vigir_control::VigirRobotFilterBase>     joint_filter_; // filter type chosen by implementation
    boost::shared_ptr<vigir_control::VigirRobotPoseFilterBase> pose_filter_;  // filter type chosen by implementation

    // NOTE: The robot specific implementation is responsible for
    //    providing data protection in multithreaded environments
    //    as these may be updated and accessed by a number of functions
    boost::shared_ptr<vigir_control::VigirRobotStateData>      current_robot_state_; // structure to store latest robot state data
    boost::shared_ptr<vigir_control::VigirRobotStateData>      filtered_robot_state_; // structure to store filtered robot state data
    boost::shared_ptr<vigir_control::VigirRobotStateData>      controlled_robot_state_; // structure to store robot control commands

    boost::shared_ptr<vigir_control::VigirRobotBehaviorData>   current_robot_behavior_;
    boost::shared_ptr<vigir_control::VigirRobotBehaviorData>   desired_robot_behavior_;

    enum InterfaceStatus{
        ROBOT_INTERFACE_OK = 0,
        ROBOT_INTERFACE_MODEL_FAILED_TO_INITIALIZE = 100,
        ROBOT_INTERFACE_API_FAILED_TO_INITIALIZE,
        ROBOT_INTERFACE_MODEL_FAILED_TO_CLEANUP_PROPERLY,
        ROBOT_INTERFACE_INTERFACE_FAILED_TO_CLEANUP_PROPERLY
    };

};

} // end of vigir_control namespace
#endif

