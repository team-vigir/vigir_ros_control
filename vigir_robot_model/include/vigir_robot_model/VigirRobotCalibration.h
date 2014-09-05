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
#ifndef __VIGIR_ROBOT_CALIBRATION_H__
#define __VIGIR_ROBOT_CALIBRATION_H__

#include <stdint.h>
#include <vigir_robot_model/VigirRobotCalibrationBase.h>

namespace vigir_control {

/**
 * This structure defines a simple linear calculation for calibration of variables.
 *
 */
struct VigirRobotCalibration : public VigirRobotCalibrationBase
{

  VigirRobotCalibration(const std::string& name,
                             const uint8_t& elements)
                : VigirRobotCalibrationBase(name,elements),
                  gearing_(elements), offset_(elements)
  {
      gearing_.fill(1.0);
      offset_.fill(0.0);
  }

  ~VigirRobotCalibration() {};

  // Apply calibration based on raw sensed data
  //    this function implicitly assumes that sensed and raw vectors are
  //    defined with the same sizes
  bool apply_calibration(VectorNd& q_sensed, VectorNd& dq_sensed, const VectorNd& q_raw, const VectorNd& dq_raw)
  {
      q_sensed   = gearing_.cwiseProduct(q_raw);
      q_sensed  += offset_;

      dq_sensed  = gearing_.cwiseProduct(dq_raw);
  }

  bool setCalibrationParameters(const VectorNd& gearing, const VectorNd& offset)
  {
      if ((gearing.size() != gearing_.size()) ||
              (offset.size() != offset_.size()))
      { // verify that the data sizes are consistent
          return false;
      }

      gearing_ = gearing;
      offset_  = offset;

      return true;
  }


  bool adjust_offset(VectorNd& diff_offset)
  {
      if (diff_offset.size() != offset_.size())
      { // verify that the data sizes are consistent
          return false;
      }

      offset_  += diff_offset;

  }

protected:
  VectorNd gearing_ ;
  VectorNd offset_;

};

} // end of vigir_control namespace
#endif

