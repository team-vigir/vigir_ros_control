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

namespace vigir_control {

    /**
    * This structure defines the vectors used to store robot joint data.
    */
    typedef struct VigirRobotJointData
    {
        VigirRobotJointData(const int32_t& n_joints = 0);

        VectorNd                        joint_positions_;
        VectorNd                        joint_velocities_;
        VectorNd                        joint_accelerations_;
        VectorNd                        joint_efforts_;
    } VigirRobotJointData;

    /* This structure holds all of the contiuous robot state data */
    typedef struct VigirRobotStateData
    {

        VigirRobotStateData(const int32_t& n_joints = 0)
            : robot_joints_(n_joints) {}

        // Internal representations of sensor data
        uint64_t                        last_update_time_;

        Pose                            pelvis_pose_;
        Vector3d                        pelvis_velocity_;
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

    } VigirRobotStateData;

    /* This structure holds all of the contiuous robot state data */
    typedef struct VigirRobotState
    {
        VigirRobotState(const int32_t& n_joints = 0)
            : current_robot_state_(n_joints),
              filtered_robot_state_(n_joints){}

        VigirRobotStateData             current_robot_state_;
        VigirRobotStateData             filtered_robot_state_;

    } VigirRobotState;

}

#endif

