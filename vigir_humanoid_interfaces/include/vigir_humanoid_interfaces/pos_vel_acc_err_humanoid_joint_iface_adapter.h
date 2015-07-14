/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
@TODO_ADD_AUTHOR_INFO
// Taken from https://github.com/gt-ros-pkg/universal_robot/blob/hydro-devel-c-api/ur_controllers/include/ur_controllers/ur_hardware_interface_adapter.h


#ifndef POS_VEL_ACC_ERR_HUMANOID_JOINT_CMD_INTERFACE_ADAPTER_H
#define POS_VEL_ACC_ERR_HUMANOID_JOINT_CMD_INTERFACE_ADAPTER_H

#include <joint_trajectory_controller/hardware_interface_adapter.h>
#include <vigir_humanoid_interfaces/pos_vel_acc_err_humanoid_joint_iface.h>

#include <control_toolbox/pid.h>



/**
 * \brief Adapter for a position/velocity/acceleration-controlled hardware interface. 
 * Forwards desired positions/velocities/accelerations as commands.
 */
template <class State>
class HardwareInterfaceAdapter<hardware_interface::PosVelAccErrHumanoidJointInterface, State>
{
public:
  HardwareInterfaceAdapter() : joint_handles_ptr_(0) {}

  bool init(std::vector<hardware_interface::PosVelAccErrHumanoidJointHandle>& joint_handles,
                        ros::NodeHandle& controller_nh)
  {
    // Store pointer to joint handles
    joint_handles_ptr_ = &joint_handles;

    // Initialize PIDs
    pids_.resize(joint_handles.size());
    for (unsigned int i = 0; i < pids_.size(); ++i)
    {
      // Node handle to PID gains
      ros::NodeHandle joint_nh(controller_nh, std::string("gains/") + joint_handles[i].getName());

      // Init PID gains from ROS parameter server
      pids_[i].reset(new control_toolbox::Pid());
      if (!pids_[i]->init(joint_nh))
      {

        ROS_WARN_STREAM("Failed to initialize PID gains from ROS parameter server for namespace "<< joint_nh.getNamespace().c_str() << " setting to defaults");
        pids_[i]->initPid(0.01, 0.0, 0.001, 0.0, 0.0, joint_nh);

        //Just to be sure, should be done in init, too
        pids_[i]->reset();
        //return false;
      }
    }

    return true;
  }

  void starting(const ros::Time& time) {}
  void stopping(const ros::Time& time) {}

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& period,
                     const State&         desired_state,
                     const State&         state_error)
  {
    const unsigned int n_joints = joint_handles_ptr_->size();
    for (unsigned int i = 0; i < n_joints; ++i) {
      (*joint_handles_ptr_)[i].setPositionCommand(desired_state.position[i]);
      (*joint_handles_ptr_)[i].setVelocityCommand(desired_state.velocity[i]);

      /*
      if ((*joint_handles_ptr_)[i].getName()=="r_arm_shx"){
        pids_[i]->printValues();
      }
      */

      double correction_acceleration = pids_[i]->computeCommand(state_error.position[i], state_error.velocity[i], period);

      (*joint_handles_ptr_)[i].setAccelerationCommand(desired_state.acceleration[i] + correction_acceleration);

      (*joint_handles_ptr_)[i].setPositionError(state_error.position[i]);
      (*joint_handles_ptr_)[i].setVelocityError(state_error.velocity[i]);
    }
  }

private:
  std::vector<hardware_interface::PosVelAccErrHumanoidJointHandle>* joint_handles_ptr_;

  typedef boost::shared_ptr<control_toolbox::Pid> PidPtr;
  std::vector<PidPtr> pids_;
};

#endif
