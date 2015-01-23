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
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <vigir_joint_interfaces/pos_vel_acc_joint_iface.h>
#include <vigir_joint_interfaces/pos_vel_acc_err_humanoid_joint_iface.h>
#include <vigir_joint_interfaces/vigir_pelvis_iface.h>

#include <vigir_robot_model/VigirRobotDataTypes.h>
#include <vigir_robot_model/VigirRobotModel.h>

namespace vigir_control {

enum VigirHumanoidSwitchMode
{
    SWITCH_DISALLOWED  =   0,
    SWITCH_IMMEDIATE   =   1,  // Switch on change request
    SWITCH_ON_FEEDBACK =   2,  // Switch on feedback matching desired state
    SWITCH_HARD_RESET  =   3,  // Stop all currently active and restart desired, even if same
    HARD_RESET_MAINTAIN=   4   // Stop and reset all controllers to maintain active but force a reset
};

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
    virtual ~VigirHumanoidHWInterface()
    {
        std::cout << "  Destroyed VigirHumanoidHWInterface!" << std::endl;
    }


    // Generic initialization functions
    virtual int32_t init_robot_controllers(boost::shared_ptr<vigir_control::VigirRobotModel>& robot_model,
                                           boost::shared_ptr<ros::NodeHandle>& behavior_control_nh,
                                           boost::shared_ptr<ros::NodeHandle>& joint_control_nh,
                                           boost::shared_ptr<ros::NodeHandle>& private_nh);

    // Generic cleanup functions
    virtual int32_t cleanup_robot_controllers();

    virtual const int32_t                          getActiveControlModeId()  = 0;
    virtual const std::vector< std::string >*      getActiveJointControllersList()= 0;
    virtual const std::vector< std::string >*      getActiveRobotControllersList()= 0;
    virtual const VigirHumanoidSwitchMode          permitControllerSwitch()  = 0;

    // Following public data structures are used directly by controllers
    // The data structures must be populated during a read() function in a
    // thread safe manner

    ros::Time                                             last_update_time_;
    boost::shared_ptr< std::vector<std::string> >         joint_names_;
    // I'd prefer to use VectorNd, but I am not sure if local_vector = data_vector
    // would ever reallocate and change memory locations after registering.
    // I would not expect it to if the underlying sizes are the same, but I'm not
    // certain, so I'm going to play it safe for now.
    //std::vector<double >                                  joint_state_positions_;
    //std::vector<double >                                  joint_state_velocities_;
    //std::vector<double >                                  joint_state_accelerations_;
    //std::vector<double >                                  joint_state_efforts_;
    VectorNd                                              joint_state_positions_;
    VectorNd                                              joint_state_velocities_;
    VectorNd                                              joint_state_accelerations_;
    VectorNd                                              joint_state_efforts_;


    VectorNd                                              joint_command_positions_;             //!< desired position
    VectorNd                                              joint_command_velocities_;            //!< desired velocity
    VectorNd                                              joint_command_accelerations_;         //!< desired acceleration
    VectorNd                                              joint_command_efforts_;               //!< desired effort
    VectorNd                                              joint_command_control_;               //!< desired control command (acceleration)
    VectorNd                                              joint_command_friction_compensation_; //!< effort to compensate for friction

    VectorNd                                              joint_position_errors_;
    VectorNd                                              joint_velocity_errors_;
    VectorNd                                              joint_effort_errors_;

    std::vector<double >                                  pelvis_states_;
    std::vector<double >                                  pelvis_commands_;
    std::vector<double >                                  pelvis_errors_;
    bool                                                  use_desired_pelvis_pose_;

  protected:
    std::string                                             name_;

    // ROS control interfaces
    hardware_interface::JointStateInterface                 joint_state_interface_;
    hardware_interface::PositionJointInterface              position_joint_interface_;
    hardware_interface::VelocityJointInterface              velocity_joint_interface_;
    hardware_interface::EffortJointInterface                effort_joint_interface_;
    hardware_interface::PosVelAccJointInterface             pos_vel_acc_joint_interface_;
    hardware_interface::PosVelAccErrHumanoidJointInterface  pos_vel_acc_err_humanoid_joint_interface_;
    //@todo RobotMode (startup) , RobotBehavior, and RobotFootsteps interfaces

    double                                                  pelvis_dummy_; // no velocity or effort for these "joints"
    std::vector<std::string>                                pelvis_joint_names_;
    hardware_interface::JointStateInterface                 pelvis_state_interface_;
    hardware_interface::VigirPelvisInterface                pelvis_command_interface_;

};

} // end of vigir_control namespace
#endif

