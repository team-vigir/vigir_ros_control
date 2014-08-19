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
#include <vigir_humanoid_controller/VigirHumanoidController.h>

namespace vigir_control {

VigirHumanoidController::VigirHumanoidController(const std::string& name)
    : name_(name)
{
    ROS_INFO("Initialize VigirHumanoidController for <%s>",name_.c_str());
}

int32_t VigirHumanoidController::run()
{
    ros::Time current_time;
    ros::Duration elapsed_time;
    ros::Time last_time = ros::Time::now();

    while (ros::ok() )//&& robot_interface_->run())
    {
        current_time = ros::Time::now();
        elapsed_time = current_time - last_time;
        last_time = current_time;

        //ROS_INFO("before read");
        robot_hw_interface_->read(current_time, elapsed_time);
        //ROS_INFO("after read");

        //ROS_INFO("before cm.update");
        cm_->update(current_time, elapsed_time);
        //ROS_INFO("after cm.update");

        //ROS_INFO("before write");
        robot_hw_interface_->write(current_time, elapsed_time);
        //ROS_INFO("after write");

    }
}
} /* namespace flor_control */
