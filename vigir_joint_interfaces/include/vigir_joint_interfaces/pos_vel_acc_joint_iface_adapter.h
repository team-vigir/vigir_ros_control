// Taken from https://github.com/gt-ros-pkg/universal_robot/blob/hydro-devel-c-api/ur_controllers/include/ur_controllers/ur_hardware_interface_adapter.h

#ifndef POS_VEL_ACC_JOINT_CMD_INTERFACE_ARAPTER_H
#define POS_VEL_ACC_JOINT_CMD_INTERFACE_ADAPTER_H

#include <joint_trajectory_controller/hardware_interface_adapter.h>
#include <vigir_joint_interfaces/pos_vel_acc_joint_iface.h>



/**
 * \brief Adapter for a position/velocity/acceleration-controlled hardware interface. 
 * Forwards desired positions/velocities/accelerations as commands.
 */
template <class State>
class HardwareInterfaceAdapter<hardware_interface::PosVelAccJointInterface, State>
{
public:
  HardwareInterfaceAdapter() : joint_handles_ptr_(0) {}

  bool init(std::vector<hardware_interface::PosVelAccJointHandle>& joint_handles, 
                        ros::NodeHandle& controller_nh)
  {
    // Store pointer to joint handles
    joint_handles_ptr_ = &joint_handles;

    return true;
  }

  void starting(const ros::Time& time) {}
  void stopping(const ros::Time& time) {}

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& /*period*/,
                     const State&         desired_state,
                     const State&         /*state_error*/)
  {
    // Forward desired position to command
    const unsigned int n_joints = joint_handles_ptr_->size();
    for (unsigned int i = 0; i < n_joints; ++i) {
      (*joint_handles_ptr_)[i].setPositionCommand(desired_state.position[i]);
      (*joint_handles_ptr_)[i].setVelocityCommand(desired_state.velocity[i]);
      (*joint_handles_ptr_)[i].setAccelerationCommand(desired_state.acceleration[i]);
    }
  }

private:
  std::vector<hardware_interface::PosVelAccJointHandle>* joint_handles_ptr_;
};

#endif

