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
#ifndef __VIGIR_ROBOT_KALMAN_FILTER_BASE_H__
#define __VIGIR_ROBOT_KALMAN_FILTER_BASE_H__

#include <stdint.h>
#include <vigir_robot_model/VigirRobotDataTypes.h>

namespace vigir_control {

/**
 * This structure defines the base class for basic
 * Kalman filtering using generic control types.
 *
 * The class assumes a vector of independent kalman filters are
 *  applied in parallel (e.g. joint state estimation).
 *
 * The filter assumes that position, velocity, and acceleration
 * are relevant; the particulars of the system model are determined
 * by the implementation files.
 *
 * Prediction
 *  xv(k+1|k) = F xv(k) + G*u(k) + v(k)
 *    v(k) ~ 0 mean , Q = E[v*v'] process noise model
 *
 *  Sensing y(k) = H*xv(k) + eta(k)
 *     eta(k) = 0 mean, W = E[eta*eta'] sensor noise covariance
 *
 *  Q process noise keeps filter from converging to much
 *      and size relative to W weights prediction vs. sensor
 *  W sensor noise should relate to actual noise
 *   W and Q represent trade offs between how fast the filter converges,
 *     and susceptibility to noise, and directly influence lag and bandwidth,
 *
 * Update
 *  xv(k+1|k+1) = xv(k+1|k) + K*nu(k+1)
 *    where nu = y(k+1) - H*qv(k+1|k)
 *    The innovation gains K are determined by the specific filter model in implementation.
 */
  struct VigirRobotKalmanFilterBase
  {

    VigirRobotKalmanFilterBase(const std::string& name,
                               const uint8_t& elements)
                  : filter_name_(name), n_elements_(elements),
                    timestamp_(0L)  {}
    virtual ~VigirRobotKalmanFilterBase() {};

    // Apply prediction step to filter
    virtual bool predict_filter(VectorNd& q, VectorNd& dq, VectorNd& ddq, const VectorNd& u, const double& dt)=0;


    // Apply correction step to filter based on sensed data
    virtual bool correct_filter(VectorNd& q, VectorNd& dq, VectorNd& ddq, const VectorNd& q_sensed, const VectorNd& dq_sensed)=0;

    std::string  filter_name_;
    int8_t       n_elements_;
    uint64_t     timestamp_;

  protected:

};

} // end of vigir_control namespace
#endif

