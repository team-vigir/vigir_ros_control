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
#include <vigir_robot_model/VigirRobotModel.h>

namespace vigir_control {


VigirRobotModel::VigirRobotModel()
    : r_foot_id_(-1), l_foot_id_(-1), r_hand_id_(-1), l_hand_id_(-1),
      b_positions_up_to_date_(false), b_kinematics_up_to_date_(false),
      b_dynamics_up_to_date_(false),  b_CoM_up_to_date_(false),
      b_transforms_up_to_date_(false)
{


}

 VigirRobotModel::~VigirRobotModel() {

 }

 uint32_t VigirRobotModel::initializeRobotJoints(const std::vector<std::string>& controlled_joints,
                            const std::string& root_link_name,
                            const std::string& l_foot_link_name,
                            const std::string& r_foot_link_name,
                            const std::string& l_hand_link_name,
                            const std::string& r_hand_link_name)
 {
     // Store the names of the specific links of concern
     root_link_name_ = root_link_name;
     rfoot_link_name_= r_foot_link_name;
     lfoot_link_name_= l_foot_link_name;
     rhand_link_name_= r_hand_link_name;
     lhand_link_name_= l_hand_link_name;

     // We will only process certain joints for modeling and control
     // For example we may ignore a small spinning LIDAR or finger joints
     // the robot stability calculations.
     n_joints_ = controlled_joints.size();

     printf(" Root(%s) Hands(%s, %s) Feet(%s, %s) n_joints=%d\n",
            root_link_name_.c_str(),
            rfoot_link_name_.c_str(),
            lfoot_link_name_.c_str(),
            rhand_link_name_.c_str(),
            lhand_link_name_.c_str(),
            n_joints_);

     // Load map of controlled joint names
     joint_map_.clear();
     //std::cout << "Loading joint map with controlled joint list ..." << std::endl;
     for (uint8_t ndx=0; ndx < n_joints_; ndx++)
     {
         //std::cout << uint32_t(ndx) << " : " << controlled_joints[ndx] << std::endl;
         joint_map_.insert(std::pair<std::string,uint8_t>(controlled_joints[ndx],ndx));
     }

     std::cout << "Joint Map size = " << joint_map_.size() << std::endl;

     if (joint_map_.size() == n_joints_)
         return 0; // all is as expected
     else
         return 1;
 }

} /* namespace flor_control */
