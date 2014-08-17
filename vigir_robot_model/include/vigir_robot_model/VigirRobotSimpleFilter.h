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
#ifndef __VIGIR_ROBOT_SIMPLE_FILTER_H__
#define __VIGIR_ROBOT_SIMPLE_FILTER_H__

#include <stdint.h>
#include <vigir_robot_model/VigirRobotFilterBase.h>

namespace vigir_control {

/**
 * This structure defines a simple pass through filter.
 * The prediction step is empty, and the correct step equates the state
 * with the latest measurement.
 *
 */
  struct VigirRobotSimpleFilter
  {

    VigirRobotSimpleFilter(const std::string& name,
                           const uint8_t& elements)
        : VigirRobotFilterBase(name, elements)  {}
    virtual ~VigirRobotSimpleFilter() {};

    // Apply prediction step to filter
    bool predict_filter(VectorNd& q, VectorNd& dq, VectorNd& ddq, const VectorNd& u, const double& dt) {};


    // Apply correction step to filter based on sensed data
    bool correct_filter(VectorNd& q, VectorNd& dq, VectorNd& ddq, const VectorNd& q_sensed, const VectorNd& dq_sensed)
    {
        assert(q.size() == q_sensed.size());
        assert(dq.size() == dq_sensed.size());

         q =  q_sensed;
        dq = dq_sensed;

    }


  protected:

};

} // end of vigir_control namespace
#endif

