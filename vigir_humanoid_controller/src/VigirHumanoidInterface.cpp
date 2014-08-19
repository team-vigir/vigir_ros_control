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

#include <stdio.h>
#include <iostream>
#include <vigir_humanoid_controller/VigirHumanoidInterface.h>

namespace vigir_control {


// Pass the ROS node handlers with defined callback queues into the interface
// for publishing and loading specific data
VigirHumanoidInterface::VigirHumanoidInterface(const std::string& name,
                       boost::shared_ptr<ros::NodeHandle>& pub_nh,
                       boost::shared_ptr<ros::NodeHandle>& private_nh)
    : name_(name),pub_nh_(pub_nh), private_nh_(private_nh)
{
    ROS_INFO("Initialize VigirHumanoidController for <%s>",name_.c_str());
}

int32_t VigirHumanoidInterface::initialize_states(const int32_t& n_joints)
{

    // Setup state vectors based on joint list size
    current_robot_state_.reset(   new VigirRobotStateData(n_joints));     ; // structure to store latest robot state data
    filtered_robot_state_.reset(  new VigirRobotStateData(n_joints)); // structure to store filtered robot state data
    controlled_robot_state_.reset(new VigirRobotStateData(n_joints)); // structure to store robot control commands

    current_robot_behavior_.reset(new VigirRobotBehaviorData());
    desired_robot_behavior_.reset(new VigirRobotBehaviorData());

    return ROBOT_INTERFACE_OK;
}

// Assign implementation specific filters to the interface
int32_t VigirHumanoidInterface::initialize_filters(boost::shared_ptr<vigir_control::VigirRobotFilterBase>& joint_filter,
                                                   boost::shared_ptr<vigir_control::VigirRobotPoseFilterBase>& pose_filter)
{
    joint_filter_ = joint_filter;
    pose_filter_  = pose_filter;
}

} /* namespace flor_control */
