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

#include <stdio.h>
#include <iostream>
#include <vigir_robot_model/VigirRobotPoseFilter.h>

namespace vigir_control {


// Implement simple pass through for robot pose data
//    we assume we have access to external data for position via fk_pose,
//     linear velocity via fk_twist, and
//     use the latest IMU based orientation and angular velocity
// No updates to the covariance matrix is performed
bool VigirRobotPoseFilter::correct_filter(Pose& pose, Twist& twist,    // Outputs
                    const IMU& imu,                                    // Robot IMU data
                    const Matrix3d * const imu_orientation_covar,
                    const Matrix3d * const imu_linear_covar ,
                    const Matrix3d * const imu_angular_covar,
                    const Pose  * const fk_pose,      const Matrix6d * const fk_pose_covar,   // Robot forward kinematics
                    const Twist * const fk_twist,     const Matrix6d * const fk_twist_covar,  //
                    const Pose  * const sensed_pose,  const Matrix6d * const sensed_pose_covar,   // External sensing
                    const Twist * const sensed_twist, const Matrix6d * const sensed_twist_covar)
{

    assert(fk_pose != NULL);
    assert(fk_twist != NULL);
    pose.position    = fk_pose->position;
    pose.orientation = imu.orientation;

    twist.linear  = fk_twist->linear;
    twist.angular = imu.angular_velocity;
    return true;
}

} /* namespace vigir_control */
