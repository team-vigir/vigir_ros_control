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

void VigirHumanoidController::error_status(const std::string& msg, int32_t rc)
{
    if (rc < 0)
    {
        ROS_ERROR("%s",msg.c_str());
        ROS_WARN("%s",msg.c_str());  // try to get on screen and in log file
        // @todo - publish status to OCS

    }
    else
    {
        // print return code with message
        ROS_ERROR((msg+" (rc=%d)").c_str(),rc);
        ROS_WARN( (msg+" (rc=%d)").c_str(),rc);  // try to get on screen and in log file
        // @todo - publish status to OCS
    }
}

// Initialization functions
int32_t VigirHumanoidController::initialize()
{
    int32_t rc;
    try{ // initialize the robot model from parameters
        rc = init_robot_model();
        if (rc)
        {
            error_status("Robot model failed to initialize",rc);
            return ROBOT_MODEL_FAILED_TO_INITIALIZE;
        }
    }
    catch(...) // @todo: catch specific exceptions and report
    {
        error_status("Robot model failed to initialize (exception)",-1);
        return ROBOT_MODEL_FAILED_TO_INITIALIZE;
    }

    try{ // Initialize the robot specific interface (defined in implementation)
        rc = init_robot_interface();
        if (rc)
        {
            error_status("Robot interface failed to initialize",rc);
            return ROBOT_INTERFACE_FAILED_TO_INITIALIZE;
        }
    }
    catch(...) // @todo: catch specific exceptions and report
    {
        error_status("Robot interface failed to initialize (exception)",-1);
        return ROBOT_INTERFACE_FAILED_TO_INITIALIZE;
    }

    try{
        rc = init_robot_behaviors();
        if (rc)
        {
            error_status("Robot behaviors failed to initialize",rc);
            return ROBOT_BEHAVIORS_FAILED_TO_INITIALIZE;
        }
    }
    catch(...) // @todo: catch specific exceptions and report
    {
        error_status("Robot behaviors failed to initialize (exception)",-1);
        return ROBOT_BEHAVIORS_FAILED_TO_INITIALIZE;
    }

    try{
        rc = init_robot_controllers();
        if (rc)
        {
            error_status("Robot controllers failed to initialize",rc);
            return ROBOT_CONTROLLERS_FAILED_TO_INITIALIZE;
        }
    }
    catch(...) // @todo: catch specific exceptions and report
    {
        error_status("Robot controllers failed to initialize (exception)", -1);
        return ROBOT_CONTROLLERS_FAILED_TO_INITIALIZE;
    }

    try{
        rc = init_robot_publishers();
        if (rc)
        {
            error_status("Robot controller publishers failed to initialize",rc);
            return ROBOT_PUBLISHERS_FAILED_TO_INITIALIZE;
        }
    }
    catch(...) // @todo: catch specific exceptions and report
    {
        error_status("Robot controller publishers failed to initialize (exception)",-1);
        return ROBOT_PUBLISHERS_FAILED_TO_INITIALIZE;
    }

    return ROBOT_INITIALIZED_OK;
}

int32_t VigirHumanoidController::cleanup()
{
    int32_t rc;
    // Generic cleanup functions
    try{
        rc = cleanup_robot_publishers();

        if (rc)
        {
            error_status("Robot publishers failed to cleanup properly",rc);
            return ROBOT_PUBLISHERS_FAILED_TO_CLEANUP_PROPERLY;
        }
    }
    catch(...) // @todo: catch specific exceptions and report
    {
        error_status("Robot  publishers failed to cleanup properly (exception)",-1);
        return ROBOT_PUBLISHERS_FAILED_TO_CLEANUP_PROPERLY;
    }

    try{
        rc = cleanup_robot_controllers();

        if (rc)
        {
            error_status("Robot controllers failed to cleanup properly",rc);
            return ROBOT_CONTROLLERS_FAILED_TO_CLEANUP_PROPERLY;
        }
    }
    catch(...) // @todo: catch specific exceptions and report
    {
        error_status("Robot  controllers failed to cleanup properly (exception)");
        return ROBOT_CONTROLLERS_FAILED_TO_CLEANUP_PROPERLY;
    }

    try{
        rc =  cleanup_robot_behaviors();
        return ROBOT_BEHAVIORS_FAILED_TO_CLEANUP_PROPERLY;

        if (rc)
        {
            error_status("Robot controllers failed to cleanup properly",rc);
            return ROBOT_CONTROLLERS_FAILED_TO_CLEANUP_PROPERLY;
        }
    }
    catch(...) // @todo: catch specific exceptions and report
    {
        error_status("Robot  controllers failed to cleanup properly (exception)");
        return ROBOT_CONTROLLERS_FAILED_TO_CLEANUP_PROPERLY;
    }

    try{
        rc =   cleanup_robot_interface();

        if (rc)
        {
            error_status("Robot interface failed to cleanup properly",rc);
            return ROBOT_INTERFACE_FAILED_TO_CLEANUP_PROPERLY;
        }
    }
    catch(...) // @todo: catch specific exceptions and report
    {
        error_status("Robot interface failed to cleanup properly (exception)");
        return ROBOT_INTERFACE_FAILED_TO_CLEANUP_PROPERLY;
    }

    try{
        rc =   cleanup_robot_model();
        if (rc)
        {
            error_status("Robot model failed to cleanup properly",rc);
            return ROBOT_MODEL_FAILED_TO_CLEANUP_PROPERLY;
        }
    }
    catch(...) // @todo: catch specific exceptions and report
    {
        error_status("Robot model failed to cleanup properly (exception)");
        return ROBOT_MODEL_FAILED_TO_CLEANUP_PROPERLY;
    }

    return ROBOT_CLEANUP_OK;

}


// Generic initialization functions
int32_t VigirHumanoidController::init_robot_model()
{
    ROS_ERROR(" Need to initialize robot model");
    return ROBOT_INITIALIZED_OK;
}

int32_t VigirHumanoidController::init_robot_behaviors()
{
    ROS_ERROR(" Need to init_robot_behaviors");
    return ROBOT_INITIALIZED_OK;
}
int32_t VigirHumanoidController::init_robot_controllers()
{
    ROS_ERROR(" Need to init_robot_controllers");
    return ROBOT_INITIALIZED_OK;
}
int32_t VigirHumanoidController::init_robot_publishers()
{
    ROS_ERROR(" Need to init_robot_publishers");
    return ROBOT_INITIALIZED_OK;
}

// Generic cleanup functions
int32_t VigirHumanoidController::cleanup_robot_model()
{
    ROS_ERROR(" Need to cleanup_robot_model");
    return ROBOT_CLEANUP_OK;
}
int32_t VigirHumanoidController::cleanup_robot_behaviors()
{
    ROS_ERROR(" Need to cleanup_robot_behaviors");
    return ROBOT_CLEANUP_OK;
}
int32_t VigirHumanoidController::cleanup_robot_controllers()
{
    ROS_ERROR(" Need to cleanup_robot_controllers");
    return ROBOT_CLEANUP_OK;
}
int32_t VigirHumanoidController::cleanup_robot_publishers()
{
    ROS_ERROR(" Need to cleanup_robot_publishers");
    return ROBOT_CLEANUP_OK;
}


} /* namespace flor_control */
