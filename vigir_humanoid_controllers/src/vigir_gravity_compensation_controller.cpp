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

#include <ros/ros.h>

#include <vigir_humanoid_controllers/vigir_gravity_compensation_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(vigir_humanoid_controllers::VigirGravityCompensationController, controller_interface::ControllerBase)

namespace vigir_humanoid_controllers
{

  bool VigirGravityCompensationController::init(hardware_interface::VigirHumanoidControllerInterface* hw, ros::NodeHandle &nh)
  {
    controller_handle_ = hw->getHandle("controller_handle");

    joint_efforts_ = *(controller_handle_.joint_command_efforts_);
    zero_vector_   = vigir_control::VectorNd::Constant(controller_handle_.robot_model_->n_joints_,0.0);

    ROS_INFO_NAMED(controller_handle_.getName(), "Gravity compensation controller %s is initialized.", controller_handle_.getName().c_str());
    return true;
  }

  // Initialize to OFF on start up
  void VigirGravityCompensationController::starting(const ros::Time& time)
  {
      ROS_WARN_NAMED(controller_handle_.getName(), "Gravity compensation controller is starting.");
  }

  // This must remain active for control, so shut off system if this unloads for any reason
  void VigirGravityCompensationController::stopping(const ros::Time& time)
  {
      ROS_WARN_NAMED(controller_handle_.getName(), "Gravity compensation controller is stopping.");
  }

  void VigirGravityCompensationController::update(const ros::Time& time, const ros::Duration& period)
  {

      // Update RBDL model with desired position, velocity, and acceleration
      //   Assuming a reasonable fast update so that desired position is close to the current position and slightly forward in time.
      controller_handle_.robot_model_->updateJointState(time.toNSec(),
                                     *(controller_handle_.joint_command_positions_),
                                     zero_vector_,
                                     zero_vector_);

      // Update the body pose to get gravity vector
      controller_handle_.robot_model_->updateBasePose(*(controller_handle_.robot_pose_));

      // Update RBDL including accelerations (here we are assuming quasi-static gravity compensation with 0.0 velocity and acceleration)
      controller_handle_.robot_model_->updateDynamics();

      // Calculate the required torques to accelearate linkages given gravity and dynamics
      controller_handle_.robot_model_->calcRequiredTorques();
      controller_handle_.robot_model_->getRequiredTorques(joint_efforts_);
      *(controller_handle_.joint_command_efforts_) += joint_efforts_;

  }

}
