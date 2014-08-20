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
  class VigirHumanoidController
  {

  public:
    VigirHumanoidController(const std::string& name, const bool& verbose=false);
    virtual ~VigirHumanoidController()
    {
        std::cout << "Destroy VigirHumanoidController ..." << std::endl;
    };

    // Initialization functions which call specific implementations
    // The node handles and associated callback queues can have the
    // same or different node handles passed to the init function
    int32_t initialize( boost::shared_ptr<ros::NodeHandle>& beh_nh,
                        boost::shared_ptr<ros::NodeHandle>& control_nh,
                        boost::shared_ptr<ros::NodeHandle>& pub_nh,
                        boost::shared_ptr<ros::NodeHandle>& private_nh);

    int32_t cleanup();

    // Main run loop of the controller -
    // this function does not exit until ROS is shutdown or the shutdown command is given
    // A basic read-update-write loop is defined, but can be overridden for
    // custom setups.
    virtual int32_t run();

    // Handle data transfer from/to the robot interface and the
    // ROS controllers HWinterface
    // NOTE: The robot specific implementation is responsible for
    //    providing data protection in multithreaded environments
    //    as these may be called at any time from the asynchronous
    //    spinner.
    virtual void read(ros::Time time, ros::Duration period) = 0;
    virtual void write(ros::Time time, ros::Duration period) = 0;

  protected:

    // generic functions given instantiated types
    int32_t init_robot_model();
    int32_t init_robot_controllers();
    int32_t cleanup_robot_controllers();

    // Implementation specific functions
    virtual int32_t init_robot_interface()    = 0;
    virtual int32_t init_robot_publishers()   = 0;

    virtual int32_t cleanup_robot_interface()   = 0;
    virtual int32_t cleanup_robot_publishers()  = 0;


    std::string                           name_;
    bool                                  verbose_;       // dump more data to logs

    // ROS stuff
    boost::shared_ptr<ros::NodeHandle>    beh_nh_;        // Handle behavior interface
    boost::shared_ptr<ros::NodeHandle>    controller_nh_; // Handle controller interface
    boost::shared_ptr<ros::NodeHandle>    pub_nh_;        // Handle controller interface
    boost::shared_ptr<ros::NodeHandle>    private_nh_;    // Private node handle

    // Interface to robot specific implementations
    boost::shared_ptr<vigir_control::VigirRobotModel>           robot_model_; // Robot model type chosen by implementation
    boost::shared_ptr<vigir_control::VigirHumanoidInterface>    robot_interface_;
    boost::shared_ptr<vigir_control::VigirHumanoidHWInterface>  robot_hw_interface_;
    boost::shared_ptr<controller_manager::ControllerManager >   cm_;

    // dump errror to screen and log and potentially publish
    virtual void error_status(const std::string& msg, int32_t rc=-1);

};

} // end of vigir_control namespace
#endif
