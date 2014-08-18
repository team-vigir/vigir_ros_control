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
#ifndef __VIGIR_HUMANOID_CONTROLLER_H__
#define __VIGIR_HUMANOID_CONTROLLER_H__

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <controller_manager/controller_manager.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

#include <joint_limits_interface/joint_limits_interface.h>

#include <vigir_humanoid_controller/VigirHumanoidHWInterface.h>


namespace vigir_control {

/**
 * This class defines the controller interface for a generic
 * humanoid robot controller.
 *
 * This class is responsible for managing the external connections to the world,
 * and the robot specific implementation is ultimately responsible for imposing a
 * particular threading model on the process.
 *
 *
 * Joint control is handled by the ros_control framwork. This class defines
 * the RobotHW interface used by the ROS controllers.
 *
 */
  class VigirHumanoidController : public hardware_interface::RobotHW
  {

  public:
    VigirHumanoidController(const std::string& name);
    virtual ~VigirHumanoidController() {};

    // Initialization functions
    virtual int32_t initialize() = 0;

    virtual int32_t cleanup()    = 0;

    // Main run loop of the controller -
    // this function does not exit until ROS is shutdown or the shutdown command is given
    int32_t run();


  protected:

    std::string                           name_;
// ROS stuff
//    boost::shared_ptr<ros::NodeHandle>    beh_nh_;        // Handle behavior interface
//    boost::shared_ptr<ros::NodeHandle>    controller_nh_; // Handle controller interface
//    boost::shared_ptr<ros::NodeHandle>    pub_nh_;        // Handle controller interface
//    ros::NodeHandle                       nhp_;           // Private node handle

//    boost::shared_ptr<ros::AsyncSpinner>  behavior_spinner_;
//    boost::shared_ptr<ros::AsyncSpinner>  controller_spinner_;
//    boost::shared_ptr<ros::AsyncSpinner>  publisher_spinner_;
//    ros::CallbackQueue                    behavior_queue_;
//    ros::CallbackQueue                    controller_queue_;
//    ros::CallbackQueue                    publisher_queue_;

    boost::shared_ptr<controller_manager::ControllerManager >   cm_;
    boost::shared_ptr<vigir_control::VigirHumanoidHWInterface>  robot_interface_;


};

} // end of vigir_control namespace
#endif

