/*=================================================================================================
// Copyright (c) 2015, David Conner, TORC Robotics
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

#include <vigir_humanoid_controllers/vigir_friction_compensation_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(vigir_humanoid_controllers::VigirFrictionCompensationController, controller_interface::ControllerBase)

namespace vigir_humanoid_controllers
{

  bool VigirFrictionCompensationController::init(hardware_interface::VigirHumanoidControllerInterface* hw, ros::NodeHandle &nh)
  {
    controller_handle_ = hw->getHandle("controller_handle");

    const std::string complete_ns = nh.getNamespace();
    std::size_t id   = complete_ns.find_last_of("/");
    name_ = complete_ns.substr(id + 1);
    ROS_INFO("   Initialize the VigirFrictionCompensationController for %s ...",name_.c_str());

    friction_stiction_slope_ = vigir_control::VectorNd::Constant(controller_handle_.robot_model_->n_joints_, 0.0);
    friction_stiction_force_ = vigir_control::VectorNd::Constant(controller_handle_.robot_model_->n_joints_, 0.0);
    friction_viscous_slope_  = vigir_control::VectorNd::Constant(controller_handle_.robot_model_->n_joints_, 0.0);

    ROS_INFO("Load linear friction model parameters");
    std::string param_string;
    std::string param;

    for (unsigned int ndx = 0; ndx < controller_handle_.robot_model_->n_joints_; ++ ndx) {

        param_string = name_ + "/joint_friction_linear/"  + controller_handle_.robot_model_->joint_names_->at(ndx);
        if (nh.searchParam(param_string,param))
        {
          if (nh.hasParam(param))
          {
            nh.getParam(param + "/stiction_slope",     friction_stiction_slope_[ndx]);
            nh.getParam(param + "/stiction_force",     friction_stiction_force_[ndx]);
            nh.getParam(param + "/viscous_slope",      friction_viscous_slope_[ndx]);
            ROS_INFO("    Friction parameters for %s :[stiction=[%f, %f] viscous=%f", controller_handle_.robot_model_->joint_names_->at(ndx).c_str(),
                          friction_stiction_slope_[ndx],
                          friction_stiction_force_[ndx],
                          friction_viscous_slope_[ndx]);
          }
          //else
          //    ROS_INFO("Skipping friction parameters for %s", param.c_str());
        }
        else
            ROS_INFO("No base friction parameters for %s", param_string.c_str());
    }


    ROS_INFO_NAMED(controller_handle_.getName(), "Friction compensation controller %s is initialized.", controller_handle_.getName().c_str());
    return true;
  }

  // Initialize to OFF on start up
  void VigirFrictionCompensationController::starting(const ros::Time& time)
  {
      ROS_WARN_NAMED(controller_handle_.getName(), "Gravity compensation controller is starting.");
  }

  // This must remain active for control, so shut off system if this unloads for any reason
  void VigirFrictionCompensationController::stopping(const ros::Time& time)
  {
      ROS_WARN_NAMED(controller_handle_.getName(), "Gravity compensation controller is stopping.");
  }

  void VigirFrictionCompensationController::update(const ros::Time& time, const ros::Duration& period)
  {

      // intialize at first call
      static vigir_control::VectorNd friction_force = *(controller_handle_.joint_state_velocities_);

      friction_force.fill(0.0);

      // Calculate force due to friction compensation
      for (unsigned int ndx = 0; ndx < controller_handle_.robot_model_->n_joints_; ++ ndx)
      {

          const double& q_dot     = (*(controller_handle_.joint_state_velocities_))[ndx];
          const double& q_dot_cmd = (*(controller_handle_.joint_command_velocities_))[ndx];

          //  *(controller_handle_.joint_command_friction_compensation_)[ndx] = 0.0;

          //skip if we do not want to move the joint
          if(std::abs(q_dot_cmd) < 0.0001)
          {
              continue;
          }

          /* if((q_dot < 0 && !q_dot_cmd < 0) || (q_dot > 0 && !q_dot_cmd > 0)) {
                 continue;
          }*/

          friction_force[ndx] = q_dot_cmd * friction_stiction_slope_[ndx];

          if(std::abs(friction_force[ndx]) > friction_stiction_force_[ndx])
          {
              friction_force[ndx] = friction_viscous_slope_[ndx] * q_dot_cmd +
                      (q_dot_cmd > 0.0 ? friction_stiction_force_[ndx] : -friction_stiction_force_[ndx] );
          }
      }

      // Assign friction compensation
      *(controller_handle_.joint_command_friction_compensation_) = friction_force;


  }

}
