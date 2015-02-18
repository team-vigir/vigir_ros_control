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
#ifndef __VIGIR_ROBOT_CALIBRATION_BASE_H__
#define __VIGIR_ROBOT_CALIBRATION_BASE_H__

#include <stdint.h>
#include <vigir_robot_model/VigirRobotDataTypes.h>

namespace vigir_control {

/**
 * This structure defines the base class for calibration of variables.
 *
 * The class assumes a vector of calibration calculations are applied in parallel
 *   (e.g. joint angle updates).
 *
 */
  struct VigirRobotCalibrationBase
  {

    VigirRobotCalibrationBase(const std::string& name,
                               const uint8_t& elements)
                  : name_(name), n_elements_(elements) {}
    virtual ~VigirRobotCalibrationBase() {};

    // Apply correction step to filter based on sensed data
    virtual bool apply_calibration(VectorNd& q_sensed, VectorNd& dq_sensed, const VectorNd& q_raw, const VectorNd& dq_raw)=0;
    virtual bool adjust_offset(VectorNd& diff_offset)=0;
    virtual bool adjust_offset(const uint8_t& jnt, const double& diff_offset)=0;

    std::string  name_;
    int8_t       n_elements_;

  protected:

};

} // end of vigir_control namespace
#endif

