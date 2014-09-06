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
#include <vigir_robot_model/VigirRobotBasic2StateKF.h>

namespace vigir_control {

VigirRobotBasic2StateKF::VigirRobotBasic2StateKF(const std::string& name,
                                     const uint8_t& elements)
              : VigirRobotFilterBase(name,elements),
                nu_q_(elements), // allocate memory
                nu_dq_(elements),
                delta_q_(elements),
                delta_dq_(elements),
                K00_(elements),
                K01_(elements),
                K10_(elements),
                K11_(elements)
{
    K00_.fill(1.0); // default to step corrections
    K01_.fill(0.0);
    K10_.fill(0.0);
    K11_.fill(1.0);
}

// Apply correction step to filter based on sensed data
bool VigirRobotBasic2StateKF::correct_filter(VectorNd& q, VectorNd& dq, VectorNd& ddq, const VectorNd& q_sensed, const VectorNd& dq_sensed)
{
    assert(q.size()  == n_elements_);
    assert(dq.size() == n_elements_);
    assert(q_sensed.size()  == n_elements_);
    assert(dq_sensed.size() == n_elements_);

    // Calculate the innovation nu = y - H*x
    nu_q_  = q_sensed  - q;
    nu_dq_ = dq_sensed - dq;

    // Calculate the filter correction -K*nu
    delta_q_   = K00_.cwiseProduct(nu_q_);
    delta_q_  +=     K01_.cwiseProduct(nu_dq_);

    delta_dq_  = K10_.cwiseProduct(nu_q_);
    delta_dq_ +=     K11_.cwiseProduct(nu_dq_);

    // Correct the filter value
    // Note: I'm not doing any sanity checks that value is within
    //       3 sigma; this should not be necessary if the K values
    //       are reasonable but that is up to the user.
    q += delta_q_;
    dq+= delta_dq_;

    return true;
}
// Update the gains used for innovation
//
bool VigirRobotBasic2StateKF::setKFInnovationGains(const VectorNd& K00, const VectorNd& K01, const VectorNd& K10,const VectorNd& K11)
{
    // Do not attempt to change filter size here
    assert(K00.size() == n_elements_);
    assert(K01.size() == n_elements_);
    assert(K10.size() == n_elements_);
    assert(K11.size() == n_elements_);

    // Single sanity check
    if (K00_.size() != n_elements_) return false;

    // Change the gains used to update the filter
    K00_ = K00;
    K01_ = K01;
    K10_ = K10;
    K11_ = K11;

    std::cout << " VigirRobotBasic2StateKF::setKFInnovationGains:" << std::endl;

    for (int i = 0; i < n_elements_; ++i)
    {
        printf(" Joint (% 2d) filter gains=[[%f, %f]; [%f, %f]]\n",
                 i, K00[i], K01[i], K10[i], K11[i]);
    }

    return true;
}

} /* namespace flor_control */
