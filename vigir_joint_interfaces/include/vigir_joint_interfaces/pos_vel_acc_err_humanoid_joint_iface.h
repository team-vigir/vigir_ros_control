
// taken from https://github.com/gt-ros-pkg/universal_robot/tree/hydro-devel-c-api/ur_ctrl_client
/// \author Kelsey Hawkins

#ifndef POS_VEL_ACC_ERR_HUMANOID_JOINT_CMD_INTERFACE_H
#define POS_VEL_ACC_ERR_HUMANOID_JOINT_CMD_INTERFACE_H

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_command_interface.h>

namespace hardware_interface
{

/** \brief A handle used to read and command a single joint. */
class PosVelAccErrHumanoidJointHandle : public JointHandle
{
public:
  PosVelAccErrHumanoidJointHandle() : hardware_interface::JointHandle(), desired_vel_(0), desired_acc_(0), pos_error_(0), vel_error_(0) {}

  /**
   * \param js This joint's handle
   * \param pos A pointer to the storage for this joint's position command
   * \param vel A pointer to the storage for this joint's velocity command
   * \param acc A pointer to the storage for this joint's acceleration command
   */
  PosVelAccErrHumanoidJointHandle(const JointStateHandle& js, double* desired_pos, double* desired_vel, double* desired_acc, double* pos_error, double* vel_error)
    : JointHandle(js, desired_pos), desired_vel_(desired_vel), desired_acc_(desired_acc), pos_error_(pos_error), vel_error_(vel_error)
  {
    if (!desired_pos || !desired_vel_ || !desired_acc_ || !pos_error_ || !vel_error_)
    {
      throw HardwareInterfaceException(
          "Cannot create handle '" + js.getName() + "'. Some command data pointer is null.");
    }
  }

  void   setPositionCommand(double pos) {setCommand(pos);} // JointHandle uses cmd_ for desired position
  double getPositionCommand() const {return getCommand();}

  void   setVelocityCommand(double vel) {assert(desired_vel_); *desired_vel_ = vel;}
  double getVelocityCommand() const {assert(desired_vel_); return *desired_vel_;}

  void   setAccelerationCommand(double acc) {assert(desired_acc_); *desired_acc_ = acc;}
  double getAccelerationCommand() const {assert(desired_acc_); return *desired_acc_;}

  void   setPositionError(double pos) {assert(pos_error_); *pos_error_ = pos;}
  double getPositionError() const {assert(pos_error_); return *pos_error_;}

  void   setVelocityError(double vel) {assert(vel_error_); *vel_error_ = vel;}
  double getVelocityError() const {assert(vel_error_); return *vel_error_;}

private:
  double* desired_vel_;
  double* desired_acc_;
  double* pos_error_;
  double* vel_error_;
};

/** \brief Hardware interface to support commanding an array of joints by position velocity and acceleration.
 *
 * \note Getting a joint handle through the getHandle() method \e will claim that resource.
 *
 */
class PosVelAccErrHumanoidJointInterface :
  public HardwareResourceManager<PosVelAccErrHumanoidJointHandle, ClaimResources> {};

}

#endif
