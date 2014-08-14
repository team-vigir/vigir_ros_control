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
#ifndef __VIGIR_ROBOT_BASIC_KALMAN_FILTER_H__
#define __VIGIR_ROBOT_BASIC_KALMAN_FILTER_H__

#include <vector>
#include <map>
#include <stdint.h>
#include <vigir_robot_model/VigirRobotKalmanFilterBase.h>

namespace vigir_control {

/**
 * This structure defines a basic Kalman filter for
 * 2-state Kalman filtering using generic control types.
 *
 *  qv=[q,qdot] with acceleration as input  uv=[0;qddot]
 *    v(k) ~ 0 mean , Q = E[v*v'] process noise model
 *
 * Prediction
 *  qv(k+1|k) = F qv(k) + G*u(k) + v(k)
 *
 *   F = [ 1  dt]    G = [ 0]  Q = noise covariance E[v*v']
 *       [ 0   1]        [dt]
 *
 *  Sensing y(k) = [ q, qdot]
 *   y(k) = H*x(k) + eta(k)
 *     eta(k) = 0 mean, W = E[eta*eta'] sensor noise covariance
 *   H = [1 0]
 *       [0 1]
 *
 *  W sensor noise should relate to actual noise
 *  Q process noise keeps filter from converging to much
 *      and size relative to W weights prediction vs. sensor
 *
 * Update
 *  qv(k+1|k+1) = qv(k+1|k) + K*nu(k+1)
 *    where nu = y(k+1) - H*qv(k+1|k)
 *
 * In this Basic form, we use a constant K matrix to apply the innovation
 * and do not track the prediction covariance.
 */
  struct VigirRobotBasicKF : public VigirRobotKalmanFilterBase
  {

    VigirRobotBasicKF(const std::string& name, const uint8_t& elements);
    virtual ~VigirRobotBasicKF(){};

    // Apply prediction step to filter
    bool predict_filter(VectorNd& q, VectorNd& dq, const VectorNd& u, const double& dt)
    {
        assert(q.size()  == n_elements_);
        assert(dq.size() == n_elements_);
        assert(u.size()  == n_elements_);

        q  += dt*dq;
        dq += dt* u;
    }


    // Apply correction step to filter based on sensed data
    bool correct_filter(VectorNd& q, VectorNd& dq, const VectorNd& q_sensed, const VectorNd& dq_sensed);

  protected:
    // Vectors to hold innovation calculation
    VectorNd nu_q_ ;
    VectorNd nu_dq_;

    // Vectors to hold correction calculation
    VectorNd delta_q_ ;
    VectorNd delta_dq_;

};

} // end of vigir_control namespace
#endif

