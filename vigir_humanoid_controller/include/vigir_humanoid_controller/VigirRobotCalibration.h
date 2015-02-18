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
#include <vigir_humanoid_controller/VigirRealTimeBuffer.h>

namespace vigir_control {

struct VigirJointCalibrationGains
{
    VigirJointCalibrationGains(const double& new_gearing=0.0, const double& new_offset=0.0)
        : gearing(new_gearing), offset(new_offset)    { }

    double gearing;
    double offset;
};

struct VigirRobotCalibrationData {

    VigirRobotCalibrationData(const int32_t& n_joints=1)
        : gearing(n_joints), offset(n_joints)
    {
        gearing.fill(1.0);
        offset.fill(0.0);
    }

    VectorNd gearing ;
    VectorNd offset;
};

/**
 * This structure defines a simple linear calculation for calibration of variables.
 *
 */
struct VigirRobotCalibration : public VigirRobotCalibrationBase
{

  VigirRobotCalibration(const std::string& name,
                        const uint8_t& elements)
                : VigirRobotCalibrationBase(name,elements),
                  calibration_data_(elements),
                  calibration_data_rt_(elements),
                  calibration_rtb_(calibration_data_,name+" calibration rtb"),
                  calibration_data_counter_(-1)
  {
      // Pass valid data to the realtime thread
      calibration_rtb_.writeBuffer(calibration_data_);
  }

  ~VigirRobotCalibration() {};

  // Apply calibration based on raw sensed data
  //    this function implicitly assumes that sensed and raw vectors are
  //    defined with the same sizes
  bool apply_calibration(VectorNd& q_sensed, VectorNd& dq_sensed, const VectorNd& q_raw, const VectorNd& dq_raw)
  {

      if (calibration_data_counter_ !=  calibration_rtb_.dataCount())
      { // Get the latest updated calibration data
          calibration_data_counter_ =  calibration_rtb_.readBuffer(calibration_data_rt_);
          ROS_INFO("Updated calibration data received  counter=%d! ", calibration_data_counter_);
      }

      q_sensed   = calibration_data_rt_.gearing.cwiseProduct(q_raw);
      q_sensed  += calibration_data_rt_.offset;

      dq_sensed  = calibration_data_rt_.gearing.cwiseProduct(dq_raw);
      return true;
  }

  bool setCalibrationParameters(const VectorNd& gearing, const VectorNd& offset)
  {
      if ((gearing.size() != calibration_data_.gearing.size()) ||
              (offset.size() != calibration_data_.offset.size()))
      { // verify that the data sizes are consistent
          return false;
      }

      calibration_data_.gearing = gearing;
      calibration_data_.offset  = offset;
      calibration_rtb_.writeBuffer(calibration_data_);

      return true;
  }


  bool adjust_offset(VectorNd& diff_offset)
  {
      if (diff_offset.size() != calibration_data_.offset.size())
      { // verify that the data sizes are consistent
          return false;
      }

      calibration_data_.offset  += diff_offset;
      calibration_rtb_.writeBuffer(calibration_data_);

      return true;
  }

  bool adjust_offset(const uint8_t& jnt, const double& diff_offset)
  {
      if (jnt < calibration_data_.offset.size())
      {
          calibration_data_.offset[jnt] += diff_offset;
          calibration_rtb_.writeBuffer(calibration_data_);
          return true;
      }
      return false;
  }

  bool update_calibration(const uint8_t& jnt, const VigirJointCalibrationGains& calibration)
  {
      if (jnt < calibration_data_.offset.size())
      {
          calibration_data_.offset[jnt]  = calibration.offset;
          calibration_data_.gearing[jnt] = calibration.gearing;
          calibration_rtb_.writeBuffer(calibration_data_);
          return true;
      }
      return false;
  }
  bool update_calibration(const uint8_t& jnt, const double& new_gearing, const double& new_offset)
  {
      if (jnt < calibration_data_.offset.size())
      {
          calibration_data_.offset[jnt]  = new_offset;
          calibration_data_.gearing[jnt] = new_gearing;
          calibration_rtb_.writeBuffer(calibration_data_);
          return true;
      }
      return false;
  }

  VigirRobotCalibrationData& getCalibrationData() { return calibration_data_;}

protected:
  VigirRobotCalibrationData                      calibration_data_;         // Data updated in non-realtime thread
  VigirRobotCalibrationData                      calibration_data_rt_;      // Data used in realtime thread for calculation
  VigirRealTimeBuffer<VigirRobotCalibrationData> calibration_rtb_;          // Buffer for transfering data to realtime thread
  int32_t                                        calibration_data_counter_; // Counter to keep track of new data

};

} // end of vigir_control namespace
#endif

