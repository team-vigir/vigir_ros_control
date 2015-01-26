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

#include <controller_manager/controller_manager.h>
#include <flor_utilities/timing.h>

#include <vigir_robot_model/VigirRobotModel.h>
#include <vigir_humanoid_controller/VigirHumanoidHWInterface.h>
#include <vigir_humanoid_controller/VigirHumanoidInterface.h>


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
      VigirHumanoidController(const std::string& name, const ros::Rate& loop_rate=ros::Rate(500), const bool& verbose=false);
    virtual ~VigirHumanoidController()
    {
        std::cout << "  Destroyed VigirHumanoidController!" << std::endl;
    }

    // Initialization functions which call specific implementations
    // The node handles and associated callback queues can have the
    // same or different node handles passed to the init function
    int32_t initialize( boost::shared_ptr<ros::NodeHandle>& mode_control_nh,
                        boost::shared_ptr<ros::NodeHandle>& joint_control_nh,
                        boost::shared_ptr<ros::NodeHandle>& robot_control_nh,
                        boost::shared_ptr<ros::NodeHandle>& pub_nh,
                        boost::shared_ptr<ros::NodeHandle>& sub_nh,
                        boost::shared_ptr<ros::NodeHandle>& private_nh);

    // This should be called before destroying the controller instance
    int32_t cleanup();


    // Main run loop of the controller -
    //  this function provides a simple single thread for repeated invocation of the update function.
    //  this function does not exit until ROS is shutdown or the shutdown command is given.
    //  this function can be overridden for custom setups.
    virtual int32_t runController();

    // Handle data transfer from/to the robot interface and the
    // ROS controllers HWinterface
    // NOTE: The robot specific implementation is responsible for
    //    providing data protection in multithreaded environments
    //    as these may be called at any time from the asynchronous
    //    spinner.
    virtual bool read(ros::Time time, ros::Duration period) = 0;
    virtual void write(ros::Time time, ros::Duration period) = 0;

    virtual bool newDataAvailable() = 0;

  protected:

    // Main update loop for controllers
    // A basic read-update each controller manager -write loop is defined, but can be overridden for custom setups.
    virtual int32_t updateController(const ros::Time&     current_time, const ros::Duration& elapsed_time);

    // generic functions given instantiated types
    int32_t init_robot_model();

    // Implementation specific functions
    virtual int32_t init_robot_controllers()  = 0;
    virtual int32_t init_robot_interface()    = 0;
    virtual int32_t init_robot_publishers()   = 0;
    virtual int32_t init_robot_subscribers()  = 0;

    virtual int32_t cleanup_robot_controllers() = 0;
    virtual int32_t cleanup_robot_interface()   = 0;
    virtual int32_t cleanup_robot_publishers()  = 0;
    virtual int32_t cleanup_robot_subscribers() = 0;


    std::string                           name_;
    bool                                  verbose_;       // dump more data to logs
    bool                                  run_flag_;
    ros::Rate                             desired_loop_rate_;
    int32_t                               active_control_mode_id_;
    const std::vector<std::string > *     active_joint_controllers_list_;
    const std::vector<std::string > *     active_robot_controllers_list_;
    bool                                  controller_switching_fault_;

    Timing                                run_loop_timing_;
    Timing                                wait_timing_;
    Timing                                read_timing_;
    Timing                                write_timing_;
    Timing                                controller_timing_;
    Timing                                mode_controller_timing_;
    Timing                                joint_controller_timing_;
    Timing                                robot_controller_timing_;
    uint32_t                              sleep_failure_;
    uint32_t                              max_read_waits_;

    // ROS stuff - these are created outside interface, and their associated callbacks and spinners determine the threading model
    boost::shared_ptr<ros::NodeHandle>    mode_controller_nh_;     // Handle control mode controller interface
    boost::shared_ptr<ros::NodeHandle>    joint_controller_nh_;    // Handle joint controller interface
    boost::shared_ptr<ros::NodeHandle>    robot_controller_nh_;    // Handle robot controller interface
    boost::shared_ptr<ros::NodeHandle>    pub_nh_;                 // Handle publisher interfaces
    boost::shared_ptr<ros::NodeHandle>    sub_nh_;                 // Handle subscriber interfaces
    boost::shared_ptr<ros::NodeHandle>    private_nh_;             // Private node handle

    // Interface to robot specific implementations
    boost::shared_ptr<vigir_control::VigirRobotModel>           robot_model_; // Robot model type chosen by implementation
    boost::shared_ptr<vigir_control::VigirHumanoidInterface>    robot_interface_;
    boost::shared_ptr<vigir_control::VigirHumanoidHWInterface>  robot_hw_interface_;

    // Independent controller managers (executed in the following order)
    boost::shared_ptr<controller_manager::ControllerManager >   mode_cm_;  // Handle control mode changes
    boost::shared_ptr<controller_manager::ControllerManager >   joint_cm_; // Handle joint level controllers (joints, ...)
    boost::shared_ptr<controller_manager::ControllerManager >   robot_cm_; // Handle robot level controllers (footsteps, whole-body, ...)

    // dump errror to screen and log and potentially publish
    virtual void error_status(const std::string& msg, int32_t rc=-1);
    virtual void cleanup_status(const std::string& msg, int32_t rc=-1);

    // Determing unique elements in old and new lists to start and stop
    void processControllerLists(const std::vector<std::string> * const old_controllers,
                                const std::vector<std::string> * const new_controllers,
                                std::vector<std::string> & stop_list, std::vector<std::string> & start_list);

};

} // end of vigir_control namespace
#endif

