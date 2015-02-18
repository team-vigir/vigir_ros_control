/*=================================================================================================
// Copyright (c) 2015, David Conner, TORC Robotics
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

#ifndef VIGIR_JOINT_CALIBRATION_GAINS_MANAGER_H
#define VIGIR_JOINT_CALIBRATION_GAINS_MANAGER_H

#include <vigir_humanoid_controller/VigirRobotCalibration.h>
#include <vigir_humanoid_controller/VigirRealTimeBuffer.h>


// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <vigir_humanoid_controller/VigirJointCalibrationGainsConfig.h>

/**
 * \brief Manager class that handles updating of calibration parameters through dynamic reconfigure and makes them safely available to real-time thread.
 */
namespace vigir_control
{
class VigirJointCalibrationGainsManager
{
public:

  VigirJointCalibrationGainsManager(const std::string& joint_name, const int& jnt_ndx, boost::shared_ptr<vigir_control::VigirRobotCalibration>& calibration) :
      joint_name_(joint_name),
      joint_index_(jnt_ndx),
      dynamic_reconfig_initialized_(false),
      calibration_(calibration){}

  inline void updateVigirJointCalibrationGainsDynamicReconfigure(vigir_control::VigirJointCalibrationGains& gains)
  {
      // Make sure dynamic reconfigure is initialized
     if(!dynamic_reconfig_initialized_)
        return;

     vigir_humanoid_controller::VigirJointCalibrationGainsConfig config;
     config.gearing      = gains.gearing   ;
     config.offset       = gains.offset    ;

     // Set starting values, using a shared mutex with dynamic reconfig
     param_reconfig_mutex_.lock();
     joint_calibration_reconfig_server_->updateConfig(config);
     param_reconfig_mutex_.unlock();
  }


  // Safely retrieve the latest data
  inline void getVigirJointCalibrationGains(vigir_control::VigirJointCalibrationGains& gains)
  {
      vigir_control::VigirRobotCalibrationData& calibration_data = calibration_->getCalibrationData();
      if (joint_index_ < calibration_data.gearing.size())
      {
          gains.gearing = calibration_data.gearing[joint_index_];
          gains.offset  = calibration_data.offset[joint_index_];
      }
      else
      {
          ROS_ERROR("Invalid joint index =%d in getVigirJointCalibrationGains ",joint_index_);
          gains.gearing = 1.0;
          gains.offset  = 0.0;
      }
  }

  // Update the thread safe buffer
  inline void setVigirJointCalibrationGains(vigir_control::VigirJointCalibrationGains& gains)
  {
      if (calibration_->update_calibration(joint_index_,gains))
      {
            ROS_INFO("      setVigirJointCalibrationGains for joint[%d] %s: gearing=%6g offset=%6g",
                     joint_index_,
                     joint_name_.c_str(),
                     gains.gearing,gains.offset);
            updateVigirJointCalibrationGainsDynamicReconfigure(gains);
      }
  }

  // Process update from dynamic reconfigure
  inline void dynamicReconfigVigirJointCalibrationGainsCallback(vigir_humanoid_controller::VigirJointCalibrationGainsConfig &config, uint32_t level)
  {
      ROS_INFO("Received dynamicReconfigVigirJointCalibrationGainsCallback for joint[%d] = %s : gearing=%6g offset=%6g!",
                joint_index_, joint_name_.c_str(), config.gearing,config.offset);

      vigir_control::VigirJointCalibrationGains gains;
      gains.gearing      = config.gearing     ;
      gains.offset       = config.offset       ;
      setVigirJointCalibrationGains(gains);
  }


  bool init(boost::shared_ptr<ros::NodeHandle>& calibration_nh)
  {

    ROS_INFO(" Inside VigirJointInterfaceControlGainsManager for joint[%d] %s",joint_index_, joint_name_.c_str());


    // Set up dynamic reconfigure for gains
    std::string calibration_data_name = calibration_->name_+"/"+joint_name_;
    ros::NodeHandle joint_calibration_nh(calibration_data_name); // create node handle for this specific joint topic
    ROS_INFO(" Creating VigirJointInterfaceControlGainsManager NodeHandle for %s  (%s)",joint_calibration_nh.getNamespace().c_str(), calibration_data_name.c_str());

    joint_calibration_reconfig_server_.reset( new VigirJointCalibrationGainsDynamicReconfigServer(param_reconfig_mutex_, joint_calibration_nh));

    joint_calibration_reconfig_callback_ = boost::bind(&VigirJointCalibrationGainsManager::dynamicReconfigVigirJointCalibrationGainsCallback, this, _1, _2);

    joint_calibration_reconfig_server_->setCallback(joint_calibration_reconfig_callback_);

    dynamic_reconfig_initialized_ = true;


    // Initialize the gains
    vigir_control::VigirJointCalibrationGains  gains;
    joint_calibration_nh.getParam("gearing" , gains.gearing);
    joint_calibration_nh.getParam("offset" ,  gains.offset );

    setVigirJointCalibrationGains(gains);

    return true;
  }



private:
  boost::shared_ptr<vigir_control::VigirRobotCalibration>    calibration_; // Pointer to the calibration class

  // Dynamics reconfigure
  typedef dynamic_reconfigure::Server<vigir_humanoid_controller::VigirJointCalibrationGainsConfig> VigirJointCalibrationGainsDynamicReconfigServer;

  boost::shared_ptr<VigirJointCalibrationGainsDynamicReconfigServer>    joint_calibration_reconfig_server_;
  VigirJointCalibrationGainsDynamicReconfigServer::CallbackType         joint_calibration_reconfig_callback_;

  bool                                                       dynamic_reconfig_initialized_;
  boost::recursive_mutex                                     param_reconfig_mutex_;
  std::string                                                joint_name_;
  int32_t                                                    joint_index_;
};

} // namespace

#endif
