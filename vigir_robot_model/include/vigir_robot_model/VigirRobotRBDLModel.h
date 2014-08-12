
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

#ifndef __VIGIR_ROBOT_RBDL_MODEL_H__
#define __VIGIR_ROBOT_RBDL_MODEL_H__


#include <vigir_robot_model/VigirRobotModel.h>
#include <boost/shared_ptr.hpp>

namespace vigir_control {

typedef std::map<std::string, double> JointMap;

struct VigirRobotRBDLPrivate; // private implementation to hide details of RBDL from users


/**
 * Class to compute the center of mass while recursively traversing
 * a kinematic tree of a robot (from URDF)
 *
 */
class VigirRobotRBDLModel : public VigirRobotModel
{
public:

  VigirRobotRBDLModel();
  virtual ~VigirRobotRBDLModel();

  /**
   *  Update kinematics
   */

  /**
    * Load model from XML-based urdf string
    */
  uint32_t loadRobotModel(const std::string xml,
                          const double& mass_factor = 1.0,
                          const bool& verbose = false);


  /**
   * Updates position and velocity and filtered state given the latest robot_state data
   * This must be called while thread protected.
   */
  void updateJointState(const uint64_t& timestamp, const VectorNd& joint_positions, const VectorNd& joint_velocities, const VectorNd& joint_accelerations);
  void updatePositions() ;  // update RBDL link positions given latest robot state
  void updateKinematics();  // update RBDL kinematics (position and velocity) given latest robot state
  void updateDynamics()  ;  // update RBDL dynamics (position, velocity, and accelerations) given latest robot state

  /** Update base joint pose and velocity wrt root frame in (tx, ty, tz, Rz, Ry, Rx) */
  void updateBasePose(const PoseZYX& pose);
  void updateBaseVelocity(const PoseZYX& velocity);

  /** Update base joint pose and velocity wrt root frame in (tx, ty, tz, quat) */
  void updateBasePose(const Pose& pose);

  void calcEETransforms();    // calculate transform of End Effectors in pelvis frame
  void calcRequiredTorques(); // Given gravity and joint accelerations
  void calcCOM();             // calc CoM given updated kinematics

  void getRequiredTorques(VectorNd& cmd_efforts);

  void getLeftHandMass(Vector3d& CoM, float& mass);
  void getRightHandMass(Vector3d& CoM, float& mass);

  void setLeftHandMass( const Vector3d& CoM, const float& mass, const Vector3d& Ix, const Vector3d& Iy, const Vector3d& Iz);
  void setRightHandMass(const Vector3d& CoM, const float& mass, const Vector3d& Ix, const Vector3d& Iy, const Vector3d& Iz);

protected:

  // RBDL
  std::vector<int8_t>             rbdl_to_ctrl_joint;
  std::vector<int8_t>             ctrl_to_rbdl_joint;

private:

  boost::shared_ptr<VigirRobotRBDLPrivate>  my_rbdl_;

};

} /* namespace vigir_control */
#endif
