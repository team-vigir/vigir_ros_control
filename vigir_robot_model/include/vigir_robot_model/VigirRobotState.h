//=================================================================================================
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

#ifndef __VIGIR_ROBOT_STATE_H__
#define __VIGIR_ROBOT_STATE_H__

#include <vector>
#include <stdint.h>
#include <vigir_robot_model/VigirRobotDataTypes.h>

#include <iostream>

namespace vigir_control {

    /**
    * This structure defines the vectors used to store robot joint data.
    */
    typedef struct VigirRobotJointData
    {
        VigirRobotJointData(const int32_t& n_joints = 0);

        uint64_t                        last_update_time_;
        VectorNd                        joint_positions_;
        VectorNd                        joint_velocities_;
        VectorNd                        joint_accelerations_;
        VectorNd                        joint_efforts_;

        // Fast copy for same size vectors
        VigirRobotJointData& operator= (const VigirRobotJointData& rhs)
        {
            last_update_time_ = rhs.last_update_time_;
            if (joint_positions_.size() == rhs.joint_positions_.size())
            {
                // Memcopy first block with fixed data sizes
                memcpy((uint8_t*)  this->joint_positions_.data(),
                       (uint8_t*)    rhs.joint_positions_.data(),
                       (sizeof(double) * joint_positions_.size()));

                memcpy((uint8_t*)  this->joint_velocities_.data(),
                       (uint8_t*)    rhs.joint_velocities_.data(),
                       (sizeof(double) * joint_velocities_.size()));

                memcpy((uint8_t*)  this->joint_accelerations_.data(),
                       (uint8_t*)    rhs.joint_accelerations_.data(),
                       (sizeof(double) * joint_accelerations_.size()));

                memcpy((uint8_t*)  this->joint_efforts_.data(),
                       (uint8_t*)    rhs.joint_efforts_.data(),
                       (sizeof(double) * joint_efforts_.size()));
            }
            else
            {
                joint_positions_       = rhs.joint_positions_;
                joint_velocities_      = rhs.joint_velocities_;
                joint_accelerations_   = rhs.joint_accelerations_;
                joint_efforts_         = rhs.joint_efforts_;
            }

            return *this;
        }

        bool operator== (const VigirRobotJointData& rhs)
        {
            if (joint_positions_.size() != rhs.joint_positions_.size()) return false;

            return ((joint_positions_     == rhs.joint_positions_) &&
                    (joint_velocities_    == rhs.joint_velocities_) &&
                    (joint_accelerations_ == rhs.joint_accelerations_) &&
                    (joint_efforts_       == rhs.joint_efforts_));

        }

        bool operator!= (const VigirRobotJointData& rhs)
        {
            if (joint_positions_.size() != rhs.joint_positions_.size()) return true;

            return ((joint_positions_     != rhs.joint_positions_) ||
                    (joint_velocities_    != rhs.joint_velocities_) ||
                    (joint_accelerations_ != rhs.joint_accelerations_) ||
                    (joint_efforts_       != rhs.joint_efforts_));

        }

    } VigirRobotJointData;

    /* This structure holds all of the contiuous robot state data */
    typedef struct VigirRobotStateData
    {

        VigirRobotStateData(const int32_t& n_joints = 0)
            : last_update_time_(0L), robot_joints_(n_joints) {}

        // Internal representations of sensor data
        uint64_t                        last_update_time_;

        Pose                            pelvis_pose_;
        Twist                           pelvis_twist_;
        Wrench                          r_foot_wrench_;
        Wrench                          l_foot_wrench_;
        Wrench                          r_hand_wrench_;
        Wrench                          l_hand_wrench_;
        IMU                             imu_data_;

       /* All robot body data stored in single vector;
        * individual appendage chains are extracted using joint maps.
        *  Joint alignment is specified by user provided joint list.
        */
        VigirRobotJointData             robot_joints_;

        bool operator== (const VigirRobotStateData& rhs)
        {
            if (robot_joints_.joint_positions_.size() != rhs.robot_joints_.joint_positions_.size()) return false;

            return ((robot_joints_    == rhs.robot_joints_) &&
                    (pelvis_pose_     == rhs.pelvis_pose_) &&
                    (pelvis_twist_    == rhs.pelvis_twist_) &&
                    (r_foot_wrench_   == rhs.r_foot_wrench_) &&
                    (r_hand_wrench_   == r_hand_wrench_ ) &&
                    (l_hand_wrench_   == l_hand_wrench_ ) &&
                    (imu_data_        == imu_data_      ) );

        }

        bool operator!= (const VigirRobotStateData& rhs)
        {
            if (robot_joints_.joint_positions_.size() != rhs.robot_joints_.joint_positions_.size()) return true;

            return ((robot_joints_    != rhs.robot_joints_) ||
                    (pelvis_pose_     != rhs.pelvis_pose_) ||
                    (pelvis_twist_    != rhs.pelvis_twist_) ||
                    (r_foot_wrench_   != rhs.r_foot_wrench_) ||
                    (r_hand_wrench_   != r_hand_wrench_ ) ||
                    (l_hand_wrench_   != l_hand_wrench_ ) ||
                    (imu_data_        != imu_data_      ) );
        }

    } VigirRobotStateData;

}

#endif

