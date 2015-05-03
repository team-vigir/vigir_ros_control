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
#include <boost/shared_ptr.hpp>

#include <vigir_robot_model/VigirRobotDataTypes.h>
#include <vigir_robot_model/VigirRobotStability.h>

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

    std::string                                      head_base_name_;
    std::string                                      rhand_link_name_;
    std::string                                      lhand_link_name_;
    std::string                                      root_link_name_;
    std::string                                      rfoot_link_name_;
    std::string                                      lfoot_link_name_;

    int8_t                                           r_foot_id_;
    int8_t                                           l_foot_id_;
    int8_t                                           r_hand_id_;
    int8_t                                           l_hand_id_;

    // Create map from joint name to vector index
    boost::shared_ptr<std::vector<std::string>  >   joint_names_;
    std::map<std::string, int8_t>                   joint_map_;
    int32_t                                         n_joints_;

    std::vector<int8_t>                             left_arm_joint_chain_;
    std::vector<int8_t>                             left_leg_joint_chain_;
    std::vector<int8_t>                             right_arm_joint_chain_;
    std::vector<int8_t>                             right_leg_joint_chain_;
    std::vector<int8_t>                             torso_joint_chain_;
    std::vector<int8_t>                             neck_joint_chain_;

    uint64_t                                        timestamp_;   // time of latest kinematic update
    double                                          mass_;
    Vector3d                                        CoM_pelvis_;  // in pelvis frame

    Transform                                       r_foot_transform_; // in pelvis frame
    Transform                                       l_foot_transform_;
    Transform                                       r_hand_transform_;
    Transform                                       l_hand_transform_;
    bool                                            b_positions_up_to_date_;
    bool                                            b_kinematics_up_to_date_;
    bool                                            b_dynamics_up_to_date_;
    bool                                            b_CoM_up_to_date_;
    bool                                            b_transforms_up_to_date_;

    VigirStabilityData                              stability_data_;

    uint32_t initializeRobotJoints(const std::vector<std::string>& controlled_joints,
                                   const std::vector<std::string>& left_arm_chain,
                                   const std::vector<std::string>& left_leg_chain,
                                   const std::vector<std::string>& right_arm_chain,
                                   const std::vector<std::string>& right_leg_chain,
                                   const std::vector<std::string>& torso_chain,
                                   const std::vector<std::string>& neck_chain,
                                   const std::string& root_link_name,
                                   const std::string& l_foot_link_name,
                                   const std::string& r_foot_link_name,
                                   const std::string& l_hand_link_name,
                                   const std::string& r_hand_link_name);


    // The following methods will depend on the specific framework used to model
    // the robot kinematics and dynamics.
    virtual uint32_t loadRobotModel(const std::string xml,
                                    const double& mass_factor = 1.0,
                                    const bool& verbose = false) =  0;

    /**
     * Updates position and velocity and filtered state given the latest robot_state data
     * This must be called while thread protected.
     */
    virtual void updateJointState(const uint64_t& timestamp, const VectorNd& joint_positions, const VectorNd& joint_velocities, const VectorNd& joint_accelerations)=0;
    virtual void updatePositions()  = 0;  // update RBDL link positions given latest robot state
    virtual void updateKinematics() = 0;  // update RBDL kinematics (position and velocity) given latest robot state
    virtual void updateDynamics()   = 0;  // update RBDL dynamics (position, velocity, and accelerations) given latest robot state

    /** Update base joint pose and velocity wrt root frame in (tx, ty, tz, Rz, Ry, Rx) */
    virtual void updateBasePose(const PoseZYX& pose) = 0;
    virtual void updateBaseVelocity(const PoseZYX& velocity) = 0;

    /** Update base joint pose and velocity wrt root frame in (tx, ty, tz, quat) */
    virtual void updateBasePose(const Pose& pose) = 0;

    // Functions to calculate various terms given updated kinematics
    virtual void calcEETransforms() = 0;    // calculate transform of End Effectors in pelvis frame
    virtual void calcRequiredTorques() = 0; // Given gravity and joint accelerations
    virtual void calcCOM() = 0;             // calc CoM given updated kinematics

    // Functions to extract parameters in generic form
    virtual void getRequiredTorques(VectorNd& cmd_efforts) = 0;

    virtual void getLeftHandMass(Vector3d& CoM, double& mass) = 0;
    virtual void getRightHandMass(Vector3d& CoM, double& mass) = 0;

    virtual void setLeftHandMass( const Vector3d& CoM, const double& mass) = 0;
    virtual void setRightHandMass(const Vector3d& CoM, const double& mass) = 0;

    virtual void setLeftHandInteria( const Vector3d& CoM, const double& mass, const Vector3d& Ix, const Vector3d& Iy, const Vector3d& Iz) = 0;
    virtual void setRightHandIntertia(const Vector3d& CoM, const double& mass, const Vector3d& Ix, const Vector3d& Iy, const Vector3d& Iz) = 0;

    virtual void getJointTransform(const int32_t& ctrl_joint_id, Transform& T) = 0;
    virtual void getBaseTransform(Transform& T) = 0;


    inline void getCoM(Vector3d& CoM, double& mass)
    {
        CoM = CoM_pelvis_;
        mass = mass_;
    };

    inline void getRightFoot(Transform& T)
    {
        T = this->r_foot_transform_;
    };
    inline void getLeftFoot(Transform& T)
    {
        T = this->l_foot_transform_;
    };
    inline void getLeftHand(Transform& T)
    {
        T = this->l_hand_transform_;
    };
    inline void getRightHand(Transform& T)
    {
        T = this->r_hand_transform_;
    };
    inline void getHands(Transform& rT, Transform& lT)
    {
        rT = r_hand_transform_;
        lT = l_hand_transform_;
    };

    inline void getFeet(Transform& rT, Transform& lT)
    {
        rT = r_foot_transform_;
        lT = l_foot_transform_;
    }

    inline void getTransforms(Transform& rfT, Transform& lfT, Transform& rhT, Transform& lhT)
    {
        rfT = r_foot_transform_;
        lfT = l_foot_transform_;
        rhT = r_hand_transform_;
        lhT = l_hand_transform_;
    }

    inline void getKinematics(uint64_t& timestamp, Vector3d& CoM, double& mass, Transform& rfT, Transform& lfT, Transform& rhT, Transform& lhT)
    {
        timestamp  = this->timestamp_;
        if (!b_CoM_up_to_date_) calcCOM();
        CoM        = this->CoM_pelvis_;
        mass       = this->mass_;
        if (!b_transforms_up_to_date_) calcEETransforms();
        rfT         = this->r_foot_transform_;
        lfT         = this->l_foot_transform_;
        rhT         = this->r_hand_transform_;
        lhT         = this->l_hand_transform_;
    }

  protected:

};

} // end of vigir_control namespace
#endif

