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
#ifndef __VIGIR_ROBOT_MODEL_H__
#define __VIGIR_ROBOT_MODEL_H__

#include <vector>
#include <map>
#include <stdint.h>
#include <vigir_robot_model/VigirRobotDataTypes.h>
#include <flor_utilities/timing.h>

namespace vigir_control {

/**
 * This structure defines the base data from the robot model.
 * The structure assumes a humanoid robot with 2 arm, 2 legs, torso, and
 * separate head joints.
 *
 * The structure provides direct access to the data, and therefore it is up
 * to the user to provide necessary protection if used in a multi-threaded
 * process.
 *
 * We assume that hand data is treated separately.
 */
  struct VigirRobotModel {

    VigirRobotModel();
    virtual ~VigirRobotModel();

    std::string                     head_base_name_;
    std::string                     rhand_link_name_;
    std::string                     lhand_link_name_;
    std::string                     root_link_name_;
    std::string                     rfoot_link_name_;
    std::string                     lfoot_link_name_;

    int8_t                          r_foot_id_;
    int8_t                          l_foot_id_;
    int8_t                          r_hand_id_;
    int8_t                          l_hand_id_;

    // Create map from joint name to vector index
    std::vector<std::string>        joint_names_;
    std::map<std::string, int8_t>   joint_map_;
    int32_t                         n_joints_;

    std::vector<int8_t>             left_arm_joint_chain_;
    std::vector<int8_t>             left_leg_joint_chain_;
    std::vector<int8_t>             right_arm_joint_chain_;
    std::vector<int8_t>             right_leg_joint_chain_;
    std::vector<int8_t>             torso_joint_chain_;
    std::vector<int8_t>             head_joint_chain_;

    Vector3d                        CoM_pelvis_; // Center of mass in pelvis frame
    double                          mass_;

    Transform                       r_foot_transform_;
    Transform                       l_foot_transform_;
    Transform                       r_hand_transform_;
    Transform                       l_hand_transform_;
    bool                            b_transforms_up_to_date;

    virtual bool loadRobotModel(const std::string xml,
                                const double& base_mass,
                                const Vector3d& base_com,
                                const Matrix3d& base_inertia,
                                const std::string& root_name,
                                const std::string& r_foot_name,
                                const std::string& l_foot_name,
                                const std::string& r_hand_name,
                                const std::string& l_hand_name,
                                const std::string& head_base_name="",
                                const double& mass_factor = 1.0) =  0;

    virtual void updateJointState(const uint64_t& timestamp, const VectorNd& joint_positions, const VectorNd& joint_velocities, const VectorNd& joint_accelerations)=0;
    virtual void updateKinematics(const Quatd& pelvis_orientation) = 0;  // update RBDL kinematics and gravity vector given latest robot state

    virtual void calcEETransforms() = 0;    // calculate transform of End Effectors in pelvis frame
    virtual void calcRequiredTorques() = 0; // Given gravity and joint accelerations
    virtual void calcCOM() = 0;             // calc CoM given updated kinematics

    virtual void getRequiredTorques(VectorNd& cmd_efforts) = 0;

    virtual void getLeftHandMass(Vector3d& CoM, float& mass) = 0;
    virtual void getRightHandMass(Vector3d& CoM, float& mass) = 0;

    virtual void setLeftHandMass( const Vector3d& CoM, const float& mass, const Vector3d& Ix, const Vector3d& Iy, const Vector3d& Iz) = 0;
    virtual void setRightHandMass(const Vector3d& CoM, const float& mass, const Vector3d& Ix, const Vector3d& Iy, const Vector3d& Iz) = 0;

  protected:
    Timing calc_inverse_gravity_timing_;
    Timing calc_ee_transforms_timing_;
    Timing com_calc_timing_;
    Timing update_kinematics_timing_;

};

} // end of vigir_control namespace
#endif

