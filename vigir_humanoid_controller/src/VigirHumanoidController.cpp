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
#include <vigir_humanoid_controller/VigirHumanoidController.h>
#include <vigir_humanoid_controller/VigirHumanoidStatusCodes.h>


// List joint names
template < typename T >
inline std::ostream& operator << (std::ostream& os, const std::vector<T>& v)
{
    os << "[";
    for (typename std::vector<T>::const_iterator ii = v.begin(); ii != v.end(); ++ii)
    {
        os << " " << *ii;
    }
    os << " ]";
    return os;
}

namespace vigir_control {

VigirHumanoidController::VigirHumanoidController(const std::string& name, const bool& verbose)
    : name_(name), verbose_(verbose)
{
    ROS_INFO("Initialize VigirHumanoidController for <%s>",name_.c_str());
}

int32_t VigirHumanoidController::run()
{
    ros::Time current_time;
    ros::Duration elapsed_time;
    ros::Time last_time = ros::Time::now();

    while (ros::ok() )//&& robot_interface_->run())
    {
        current_time = ros::Time::now();
        elapsed_time = current_time - last_time;
        last_time = current_time;

        //ROS_INFO("before read");
        this->read(current_time, elapsed_time);
        //ROS_INFO("after read");

        //ROS_INFO("before cm.update");
        cm_->update(current_time, elapsed_time);
        //ROS_INFO("after cm.update");

        //ROS_INFO("before write");
        this->write(current_time, elapsed_time);
        //ROS_INFO("after write");

    }
}

// Initialization functions
int32_t VigirHumanoidController::initialize(boost::shared_ptr<ros::NodeHandle>& beh_nh,
                                            boost::shared_ptr<ros::NodeHandle>& control_nh,
                                            boost::shared_ptr<ros::NodeHandle>& pub_nh,
                                            boost::shared_ptr<ros::NodeHandle>& private_nh)
{
    beh_nh_         = beh_nh ;
    controller_nh_  = control_nh;
    pub_nh_         = pub_nh    ;
    private_nh_     = private_nh;

    int32_t rc;
    try{ // initialize the robot model from parameters
        rc = init_robot_model();
        if (rc)
        {
            error_status("Robot model failed to initialize",rc);
            return ROBOT_MODEL_FAILED_TO_INITIALIZE;
        }
    }
    catch(...) // @todo: catch specific exceptions and report
    {
        error_status("Robot model failed to initialize (exception)",ROBOT_EXCEPTION_MODEL_FAILED_TO_INITIALIZE);
        return ROBOT_EXCEPTION_MODEL_FAILED_TO_INITIALIZE;
    }

    try{ // Initialize the robot specific interface (defined in implementation)
        rc = init_robot_interface();
        if (rc)
        {
            error_status("Robot interface failed to initialize",rc);
            return ROBOT_INTERFACE_FAILED_TO_INITIALIZE;
        }
    }
    catch(...) // @todo: catch specific exceptions and report
    {
        error_status("Robot interface failed to initialize (exception)",ROBOT_EXCEPTION_INTERFACE_FAILED_TO_INITIALIZE);
        return ROBOT_EXCEPTION_INTERFACE_FAILED_TO_INITIALIZE;
    }

    try{
        rc = init_robot_publishers();
        if (rc)
        {
            error_status("Robot publishers failed to initialize",rc);
            return ROBOT_PUBLISHERS_FAILED_TO_INITIALIZE;
        }
    }
    catch(...) // @todo: catch specific exceptions and report
    {
        error_status("Robot publishers failed to initialize (exception)",ROBOT_EXCEPTION_PUBLISHERS_FAILED_TO_INITIALIZE);
        return ROBOT_EXCEPTION_PUBLISHERS_FAILED_TO_INITIALIZE;
    }

    try{
        rc = init_robot_controllers();
        if (rc)
        {
            error_status("Robot behaviors failed to initialize",rc);
            return ROBOT_CONTROLLERS_FAILED_TO_INITIALIZE;
        }
    }
    catch(...) // @todo: catch specific exceptions and report
    {
        error_status("Robot behaviors failed to initialize (exception)",ROBOT_EXCEPTION_CONTROLLERS_FAILED_TO_INITIALIZE);
        return ROBOT_EXCEPTION_CONTROLLERS_FAILED_TO_INITIALIZE;
    }
}



int32_t VigirHumanoidController::cleanup()
{
    int32_t rc;
    try{
        rc =   cleanup_robot_controllers();

        if (rc)
        {
            error_status("Robot interface failed to cleanup properly",rc);
            return ROBOT_INTERFACE_FAILED_TO_CLEANUP_PROPERLY;
        }
    }
    catch(...) // @todo: catch specific exceptions and report
    {
        error_status("Robot interface failed to cleanup properly (exception)");
        return ROBOT_INTERFACE_FAILED_TO_CLEANUP_PROPERLY;
    }

    try{
        rc =   cleanup_robot_interface();
        if (rc)
        {
            error_status("Robot interface failed to cleanup properly",rc);
            return ROBOT_INTERFACE_FAILED_TO_CLEANUP_PROPERLY;
        }
    }
    catch(...) // @todo: catch specific exceptions and report
    {
        error_status("Robot interface failed to cleanup properly (exception)");
        return ROBOT_INTERFACE_FAILED_TO_CLEANUP_PROPERLY;
    }

}

void VigirHumanoidController::error_status(const std::string& msg, int32_t rc)
{
    if (rc < 0)
    {
        ROS_ERROR("%s",msg.c_str());
        ROS_WARN("%s",msg.c_str());  // try to get on screen and in log file
        // @todo - publish status to OCS

    }
    else
    {
        // print return code with message
        ROS_ERROR((msg+" (rc=%d)").c_str(),rc);
        ROS_WARN( (msg+" (rc=%d)").c_str(),rc);  // try to get on screen and in log file
        // @todo - publish status to OCS
    }
}

// Implementation specific functions
int32_t VigirHumanoidController::init_robot_model()
{

    if (!robot_model_)
    {
        std::cerr << "Invalid robot model -not initialized!" << std::endl;
        return ROBOT_MODEL_NOT_DEFINED;
    }

    // Load robot model from parameters
    std::cout << "Reading xml file from parameter server" << std::endl;
    std::string urdf_xml, full_urdf_xml,xml_result;
    urdf_xml = std::string("robot_description");
    if (!private_nh_->searchParam(urdf_xml,full_urdf_xml))
    {
        std::cerr << "Failed to find robot_description on parameter server!" << std::endl;
        return ROBOT_MODEL_NO_ROBOT_DESCRIPTION;
    }

    if (!private_nh_->getParam(full_urdf_xml, xml_result))
    {
        std::cerr << "Failed to load the robot urdf from parameter server for test!" << std::endl;
        return ROBOT_MODEL_ROBOT_DESCRIPTION_FAILED_TO_LOAD;
    }
    //std::cout << " URDF:\n" << xml_result << std::endl;

    // Load joint list from parameters
    if (!private_nh_->searchParam("controlled_joints",full_urdf_xml))
    {
        std::cerr << "Failed to find controlled joints list on parameter server!" << std::endl;
        return ROBOT_MODEL_NO_CONTROLLED_JOINTS_LIST;
    }

    std::vector<std::string> controlled_joints;
    if (!private_nh_->getParam(full_urdf_xml, controlled_joints))
    {
        std::cerr << "Failed to load the controlled joint list for test!" << std::endl;
        return ROBOT_MODEL_CONTROLLED_JOINTS_LIST_FAILED_TO_LOAD;
    }
    std::cout << " Controlled joints:\n" << controlled_joints << std::endl;

    // Load model from URDF
    int32_t rc;
    if (rc = robot_model_->initializeRobotJoints(controlled_joints,
                                               "pelvis",
                                               "l_foot", "r_foot",   // @todo - make these parameter names
                                               "l_hand", "r_hand"))
    {
        std::cerr << "Robot model initialization failed" << std::endl;
        return ROBOT_MODEL_CONTROLLED_JOINTS_FAILED_TO_INITIALIZE;
    }

    if (uint32_t rc = robot_model_->loadRobotModel(xml_result, 1.0,verbose_))
    {
        printf("Failed to load the robot model (rc=%d)- abort!\n",rc);
        return ROBOT_MODEL_FAILED_TO_INITIALIZE;
    }
    else
    {
        printf("Successfully loaded the robot URDF model!\n");
    }

    return ROBOT_INITIALIZED_OK;
}

int32_t VigirHumanoidController::init_robot_controllers()
{
    // Set up the controller manager
    cm_.reset(new controller_manager::ControllerManager(robot_hw_interface_.get(), *controller_nh_.get()));

    // Initialize the controllers
    return robot_hw_interface_->init_robot_controllers(robot_model_->joint_names_, controller_nh_, private_nh_);

}

int32_t VigirHumanoidController::cleanup_robot_controllers()
{
    return robot_hw_interface_->cleanup_robot_controllers( );
}

} /* namespace flor_control */
