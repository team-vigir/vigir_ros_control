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
#ifndef __VIGIR_ROBOT_POSE_FILTER_BASE_H__
#define __VIGIR_ROBOT_POSE_FILTER_BASE_H__

#include <vector>
#include <map>
#include <stdint.h>
#include <vigir_robot_model/VigirRobotDataTypes.h>

namespace vigir_control {

/**
 * This structure defines the base class for basic
 * filtering for robot pose data.
 *
 * The base assumes the common predict/correct cycle of standard Kalman filter
 */
  struct VigirRobotPoseFilterBase
  {

    VigirRobotPoseFilterBase(const std::string& name)
        : filter_name_(name),timestamp_(0L)
    {
        pose_covariance_.setIdentity();
        pose_covariance_ *= 0.01;
        twist_covariance_.setIdentity();
        twist_covariance_ *= 0.01;
    }
    virtual ~VigirRobotPoseFilterBase() {}

    // Apply prediction step to filter
    virtual bool predict_filter(Pose& pose, Twist& twist,       // Outputs
                                const Twist& u,                 // Input (e.g. predicted motion)
                                const Matrix6d * const u_covar, //    optional input covariance
                                const double& dt)=0;


    // Apply correction step to filter based on sensed data
    virtual bool correct_filter(Pose& pose, Twist& twist,                               // Outputs
                                const IMU& imu,                                         // Robot IMU data
                                const Matrix3d * const imu_orientation_covar  = NULL,
                                const Matrix3d * const imu_linear_covar   = NULL,
                                const Matrix3d * const imu_angular_covar  = NULL,
                                const Pose  * const fk_pose  = NULL,      const Matrix6d * const fk_pose_covar  = NULL,   // Robot forward kinematics
                                const Twist * const fk_twist = NULL,      const Matrix6d * const fk_twist_covar  = NULL,  //
                                const Pose  * const sensed_pose  = NULL,  const Matrix6d * const sensed_pose_covar  = NULL,   // External sensing
                                const Twist * const sensed_twist = NULL , const Matrix6d * const sensed_twist_covar  = NULL)=0;

    // Get latest covariance data
    const Matrix6d& getPoseCovariance(){ return pose_covariance_;}
    const Matrix6d& getTwistCovariance(){ return twist_covariance_;}

    // Allow external initialization of the covariance matrices
    inline void setPoseCovariance(const Matrix6d& pose_covar){ pose_covariance_ = pose_covar;}
    inline void setTwistCovariance(const Matrix6d& twist_covar){ twist_covariance_ = twist_covar;}

  public:

    std::string  filter_name_;
    uint64_t     timestamp_;

  protected:

    // Define matrices to store the state covariance data
    Matrix6d pose_covariance_;
    Matrix6d twist_covariance_;


};

} // end of vigir_control namespace
#endif

