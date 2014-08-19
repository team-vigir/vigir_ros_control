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
#ifndef __VIGIR_HUMANOID_HW_INTERFACE_H__
#define __VIGIR_HUMANOID_HW_INTERFACE_H__

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include <vigir_humanoid_controller/VigirHumanoidInterface.h>


namespace vigir_control {

/**
 * This class defines the HW Interface for a generic
 * humanoid robot.
 *
 * This class is responsible for managing data transfer between
 * external processes and the robot, and defines the common interfaces.
 *
 * Joint control is handled by the ros_control framwork. This class defines
 * the RobotHW interface used by the ROS controllers.
 *
 * This code accepts ROS node handles that may or may NOT have separate ROS spinners and
 * callback queues to handle external interfaces including footsteps, and behaviors.
 * Beyond this, no particular threading model is assumed.
 * The user must manage data protection in the robot specific implementation classes
 * that determine the specific threading model used by the hardware API.
 */
  class VigirHumanoidHWInterface : public hardware_interface::RobotHW
  {

  public:
    VigirHumanoidHWInterface(const std::string& name);
    virtual ~VigirHumanoidHWInterface() {};

    // Initialization functions
    int32_t initialize(boost::shared_ptr<ros::NodeHandle>& beh_nh,
                       boost::shared_ptr<ros::NodeHandle>& control_nh,
                       boost::shared_ptr<ros::NodeHandle>& pub_nh,
                       boost::shared_ptr<ros::NodeHandle>& private_nh);

    int32_t cleanup();


    // Interface to the robot data provided to the ROS controllers
    // NOTE: The robot specific implementation is responsible for
    //    providing data protection in multithreaded environments
    //    as these may be called at any time from the asynchronous
    //    spinner.
    void read(ros::Time time, ros::Duration period)
    {
        read_state_data();
        read_behavior_data();
    }

    void write(ros::Time time, ros::Duration period)
    {
        write_behavior_data();
        write_controller_data();
    }

    enum InitializationFailures{
        ROBOT_INITIALIZED_OK = 0,
        ROBOT_MODEL_FAILED_TO_INITIALIZE,
        ROBOT_INTERFACE_FAILED_TO_INITIALIZE,
        ROBOT_BEHAVIORS_FAILED_TO_INITIALIZE,
        ROBOT_CONTROLLERS_FAILED_TO_INITIALIZE,
        ROBOT_PUBLISHERS_FAILED_TO_INITIALIZE
    };
    enum CleanupFailures{
        ROBOT_CLEANUP_OK = 0,
        ROBOT_MODEL_FAILED_TO_CLEANUP_PROPERLY,
        ROBOT_INTERFACE_FAILED_TO_CLEANUP_PROPERLY   ,
        ROBOT_BEHAVIORS_FAILED_TO_CLEANUP_PROPERLY,
        ROBOT_CONTROLLERS_FAILED_TO_CLEANUP_PROPERLY,
        ROBOT_PUBLISHERS_FAILED_TO_CLEANUP_PROPERLY
    };


  protected:

    // Generic initialization functions
    int32_t init_robot_behaviors();
    int32_t init_robot_controllers();
    int32_t init_robot_publishers();

    // Generic cleanup functions
    int32_t cleanup_robot_behaviors();
    int32_t cleanup_robot_controllers();
    int32_t cleanup_robot_publishers();

    // Implementation specific functions
    virtual int32_t init_robot_model()        = 0;
    virtual int32_t init_robot_filters()      = 0;
    virtual int32_t init_robot_calibration()  = 0;
    virtual int32_t init_robot_interface()    = 0;

    virtual int32_t cleanup_robot_interface() = 0;

    // The robot specific implementation is responsible for providing data protection
    // between robot API and joint controllers/ROS callbacks
    virtual void read_state_data( )       = 0; // to  controllers
    virtual void write_controller_data( ) = 0; // from controllers
    virtual void read_behavior_data( )    = 0; // to   controllers
    virtual void write_behavior_data( )   = 0; // from controllers

    // Define generic callbacks to ROS subscriber interfaces
    //  (other than controller interfaces)
    // setDesiredBehaviorCB
    // updateFootstepPlanCB
    // setDevicePowerCB - power on/off to hands/sensors
    // resetRobotInternalPoseBD <future>

    // Define generic publishers
    // current pose
    // current state
    // current behaviors
    // current footsteps

    std::string                           name_;

    // ROS stuff
    boost::shared_ptr<ros::NodeHandle>    beh_nh_;        // Handle behavior interface
    boost::shared_ptr<ros::NodeHandle>    controller_nh_; // Handle controller interface
    boost::shared_ptr<ros::NodeHandle>    pub_nh_;        // Handle controller interface
    boost::shared_ptr<ros::NodeHandle>    private_nh_;    // Private node handle

    // Interface to common data and robot specific hardware interface
    boost::shared_ptr<vigir_control::VigirHumanoidInterface>  robot_interface_;
    boost::shared_ptr<vigir_control::VigirRobotModel>         robot_model_; // Robot model type chosen by implementation

    // ROS control interfaces
    hardware_interface::JointStateInterface     joint_state_interface_;
    hardware_interface::PositionJointInterface  position_joint_interface_;
    hardware_interface::VelocityJointInterface  velocity_joint_interface_;
    hardware_interface::EffortJointInterface    effort_joint_interface_;

    // Following data structures are used directly by controllers
    // For singled thread applications, these can be same pointers as the
    // relevant robot_interface_ data structures;
    // For multithread applications, these are likely different memory
    // locations to provide data protection.
    // These pointer values should be determined by the specific implementation
    // in the init_robot_interface function.
    boost::shared_ptr<vigir_control::VigirRobotStateData>      current_robot_state_;    // structure to store robot state used by controllers
    boost::shared_ptr<vigir_control::VigirRobotStateData>      commanded_robot_state_;

    boost::shared_ptr<vigir_control::VigirRobotBehaviorData>   current_robot_behavior_;
    boost::shared_ptr<vigir_control::VigirRobotBehaviorData>   commanded_robot_behavior_;

    void error_status(const std::string& msg, int32_t rc=-1);

    // Define common publishers
    // error status
    // pose
    //
};

} // end of vigir_control namespace
#endif

