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
#include <vigir_humanoid_controller/VigirHumanoidHWInterface.h>
#include <vigir_humanoid_controller/VigirHumanoidStatusCodes.h>

template < typename T >
inline std::ostream& operator << (std::ostream& os, const std::vector<T>& v)
{
    os << "[";
    for (typename std::vector<T>::const_iterator ii = v.begin(); ii != v.end(); ++ii)
    {
        os << " " << *ii;
    }
    os << " ]";
    return os;
}

namespace vigir_control {

VigirHumanoidHWInterface::VigirHumanoidHWInterface(const std::string& name)
    : name_(name),last_update_time_(0L)
{
    ROS_INFO("Initialize VigirHumanoidHWInterface for <%s>",name_.c_str());
}

// Set up the data for ros_controllers
int32_t VigirHumanoidHWInterface::init_robot_controllers(boost::shared_ptr<vigir_control::VigirRobotModel>& robot_model, //boost::shared_ptr< std::vector<std::string> >& joint_list,
                                                         boost::shared_ptr<ros::NodeHandle>& behavior_control_nh,
                                                         boost::shared_ptr<ros::NodeHandle>& joint_control_nh,
                                                         boost::shared_ptr<ros::NodeHandle>& private_nh)
{
    ROS_INFO(" Initialize the generic humanoid HW interfaces ...");
    // Store the list of controlled joints
    joint_names_ = robot_model->joint_names_;
    try {
        // State inputs
        std::cout << "Initialize HW interface for " << joint_names_->size() << " joints!" << std::endl;
        joint_state_positions_      = vigir_control::VectorNd::Constant(joint_names_->size(), 0.0);//.resize( joint_names_->size());
        joint_state_velocities_     = vigir_control::VectorNd::Constant(joint_names_->size(), 0.0);//.resize(joint_names_->size());
        joint_state_accelerations_  = vigir_control::VectorNd::Constant(joint_names_->size(), 0.0);//.resize(joint_names_->size());
        joint_state_efforts_        = vigir_control::VectorNd::Constant(joint_names_->size(), 0.0);//.resize(joint_names_->size());

        // Control outputs
        joint_command_positions_              = vigir_control::VectorNd::Constant(joint_names_->size(), 0.0);
        joint_command_velocities_             = vigir_control::VectorNd::Constant(joint_names_->size(), 0.0);
        joint_command_accelerations_          = vigir_control::VectorNd::Constant(joint_names_->size(), 0.0);
        joint_command_control_                = vigir_control::VectorNd::Constant(joint_names_->size(), 0.0);
        joint_command_efforts_                = vigir_control::VectorNd::Constant(joint_names_->size(), 0.0);
        joint_command_friction_compensation_  = vigir_control::VectorNd::Constant(joint_names_->size(), 0.0);

        joint_position_errors_        = vigir_control::VectorNd::Constant(joint_names_->size(), 0.0);
        joint_velocity_errors_        = vigir_control::VectorNd::Constant(joint_names_->size(), 0.0);
        joint_effort_errors_          = vigir_control::VectorNd::Constant(joint_names_->size(), 0.0);

    }
    catch(...)
    {
        std::cerr << "Failed to allocate memory for controllers!" << std::endl;
        return ROBOT_CONTROLLERS_FAILED_TO_INITIALIZE;
    }

    for (size_t i = 0; i < joint_names_->size(); ++i){

      hardware_interface::JointStateHandle state_handle(joint_names_->at(i), &joint_state_positions_[i], &joint_state_velocities_[i], &joint_state_efforts_[i]);
      joint_state_interface_.registerHandle(state_handle);

      hardware_interface::JointHandle pos_handle(joint_state_interface_.getHandle(joint_names_->at(i)), &joint_command_positions_[i]);
      position_joint_interface_.registerHandle(pos_handle);

      hardware_interface::JointHandle vel_handle(joint_state_interface_.getHandle(joint_names_->at(i)), &joint_command_velocities_[i]);
      velocity_joint_interface_.registerHandle(vel_handle);

      hardware_interface::JointHandle effort_handle(joint_state_interface_.getHandle(joint_names_->at(i)), &joint_command_efforts_[i]);
      effort_joint_interface_.registerHandle(effort_handle);

      hardware_interface::PosVelAccJointHandle pos_vel_acc_handle (joint_state_interface_.getHandle(joint_names_->at(i)),
                                                                   &joint_command_positions_[i], &joint_command_velocities_[i], &joint_command_accelerations_[i]);
      pos_vel_acc_joint_interface_.registerHandle(pos_vel_acc_handle);

      hardware_interface::PosVelAccErrHumanoidJointHandle handle (joint_state_interface_.getHandle(joint_names_->at(i)),
                                                                  &joint_command_positions_[i], &joint_command_velocities_[i], &joint_command_accelerations_[i],
                                                                  &joint_position_errors_[i], &joint_velocity_errors_[i]);

      pos_vel_acc_err_humanoid_joint_interface_.registerHandle(handle);

    }

    try {
        ROS_INFO(" Load the whole robot controller handle ...");

        hardware_interface::VigirHumanoidControllerHandle controller_handle(std::string("controller_handle"),
                                                                    &joint_state_positions_,
                                                                    &joint_state_velocities_,
                                                                    &joint_state_accelerations_,
                                                                    &joint_state_efforts_,
                                                                    &joint_command_positions_,             //!< desired position
                                                                    &joint_command_velocities_,            //!< desired velocity
                                                                    &joint_command_accelerations_,         //!< desired acceleration
                                                                    &joint_command_efforts_,               //!< desired effort
                                                                    &joint_command_control_,               //!< desired control command (acceleration)
                                                                    &joint_command_friction_compensation_, //!< effort to compensate for friction
                                                                    &joint_position_errors_,
                                                                    &joint_velocity_errors_,
                                                                    &joint_effort_errors_,
                                                                    &robot_l_foot_wrench_,
                                                                    &robot_r_foot_wrench_,
                                                                    &robot_l_hand_wrench_,
                                                                    &robot_r_hand_wrench_,
                                                                    &robot_pose_,
                                                                    robot_model);
        humanoid_controller_interface_.registerHandle(controller_handle);

        ROS_INFO(" Register the controller interface...");
        registerInterface(&humanoid_controller_interface_);
    }
    catch(...)
    {
        ROS_ERROR("Exception: could not initialize the Atlas pelvis control interface");
        return ROBOT_EXCEPTION_CONTROLLERS_FAILED_TO_INITIALIZE;
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&effort_joint_interface_);
    registerInterface(&pos_vel_acc_joint_interface_);
    registerInterface(&pos_vel_acc_err_humanoid_joint_interface_);

    ROS_INFO(" Done initializing the base HWInterface class for the humanoid controller ...");
    return ROBOT_INITIALIZED_OK;
}

int32_t VigirHumanoidHWInterface::cleanup_robot_controllers()
{
    std::cout << "      Cleanup       VigirHumanoidHWInterface::cleanup_robot_controllers!" << std::endl;
    return ROBOT_CLEANUP_OK;
}


} /* namespace vigir_control */
