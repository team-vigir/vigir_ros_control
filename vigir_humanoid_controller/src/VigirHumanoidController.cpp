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
#include <iostream>
#include <algorithm>
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

VigirHumanoidController::VigirHumanoidController(const std::string& name, const ros::Rate& loop_rate, const bool& verbose)
    : name_(name), desired_loop_rate_(loop_rate), verbose_(verbose),run_flag_(true),
      run_loop_timing_(name+" RunLoop",true,false),
      wait_timing_(name+" Wait",true,false),
      read_timing_(name+" Read",true,false),
      write_timing_(name+" Write",true,false),
      controller_timing_(name+" Controller",true,false),
      sleep_failure_(0),
      active_control_mode_id_(-1)
{
    ROS_INFO("Initialize VigirHumanoidController for <%s>",name_.c_str());
    if (desired_loop_rate_.expectedCycleTime().toSec() > 0.999)
    {
        ROS_ERROR(" Run loop coding expects loop rate to be > 1 hz - cannot run with loop < 1hz");
        exit(-1);
    }
}

int32_t VigirHumanoidController::run()
{
    ros::Time     current_time;
    ros::Duration elapsed_time;
    ros::Time     last_time = ros::Time::now();
    timespec      sleep_time;
    timespec      remaining_time;
    std::vector<std::string > running_controllers_list;

    ROS_INFO("Start controller %s run loop ...",name_.c_str());
    sleep_time.tv_sec = 0;

    while (ros::ok() && run_flag_)
    {
        current_time = ros::Time::now();
        elapsed_time = current_time - last_time;
        last_time = current_time;

        //ROS_INFO("before read");
        {
            DO_TIMING(read_timing_); // includes wait time
            if (this->read(current_time, elapsed_time))
            {
              ROS_INFO("Read data from robot - ready to begin control loop!");
              break;
            }
            else
            {
              ROS_INFO_THROTTLE(1.0,"Waiting for robot connection before starting control loop!");
            }
        }
    }

    // Start the mode controllers once we have a connection to the robot and have read valid data
    current_time = ros::Time::now();
    elapsed_time = current_time - last_time;
    last_time = current_time;
    ROS_INFO("Ready to start the mode controller manager controllers ...");
    mode_cm_->update(current_time, elapsed_time);

    mode_cm_->getRunningControllersListRealTime(running_controllers_list); // get list of all controllers currently loaded in this manager
    std::cout << "Currently Running: " << running_controllers_list << std::endl;

    mode_cm_->getControllerNamesRealtime(running_controllers_list); // get list of all controllers currently loaded in this manager
    if (!mode_cm_->switchControllerRealtime(running_controllers_list, std::vector<std::string>(),current_time, controller_manager_msgs::SwitchController::Request::STRICT))
    {
        ROS_ERROR("Failed to start the mode controllers!");
        exit(-1);
    }

    // Force starting the correct robot controllers
    active_control_mode_id_ = -1;

    // Main control loop
    ROS_INFO("Entering the main control loop...");
    while (ros::ok() && run_flag_)
    {
        {
            DO_TIMING(run_loop_timing_);
            current_time = ros::Time::now();
            elapsed_time = current_time - last_time;
            last_time = current_time;

            //ROS_INFO("before read");
            {
                DO_TIMING(read_timing_); // includes wait time
                this->read(current_time, elapsed_time);
            }
            //ROS_INFO("after read");

            //ROS_INFO("before cm.update");
            {
                DO_TIMING(controller_timing_);
                mode_cm_->update(current_time, elapsed_time);

                // Switch controllers on/off based on behavior mode
                if (robot_hw_interface_->getActiveControlModeId() != active_control_mode_id_)
                {
                    VigirHumanoidSwitchMode switch_status= robot_hw_interface_->permitControllerSwitch();
                    if (switch_status)
                    {
                        ROS_INFO("  ControlModeID changed = %d old=%d", robot_hw_interface_->getActiveControlModeId(), active_control_mode_id_);

                        robot_cm_->getRunningControllersListRealTime(running_controllers_list);

                        active_control_mode_id_ = robot_hw_interface_->getActiveControlModeId();

                        switch(switch_status)
                        {
                        case SWITCH_HARD_RESET:
                            // force hard reset by stopping all, then restarting desired controllers         start_list                              stop_list
                            controller_switching_fault_ = robot_cm_->switchControllerRealtime(*robot_hw_interface_->getActiveControllersList(), running_controllers_list, current_time, controller_manager_msgs::SwitchController::Request::BEST_EFFORT);
                            break;
                        default:
                            std::vector<std::string> stop_list;
                            std::vector<std::string> start_list;

                            // Determine unused controllers to stop, and new controllers to start (leave common controllers running)
                            processControllerLists(&running_controllers_list, robot_hw_interface_->getActiveControllersList(), stop_list, start_list);
                            controller_switching_fault_ = robot_cm_->switchControllerRealtime(start_list, stop_list, current_time, controller_manager_msgs::SwitchController::Request::BEST_EFFORT);
                        }

                        // Update active list
                        active_controllers_list_ = robot_hw_interface_->getActiveControllersList();
                    }
                }

                robot_cm_->update(current_time, elapsed_time);
            }
            //ROS_INFO("after cm.update");

            //ROS_INFO("before write");
            {
                DO_TIMING(write_timing_);
                this->write(current_time, elapsed_time);
            }
            //ROS_INFO("after write");

            ros::Time end_time = ros::Time::now();
            elapsed_time = end_time - current_time;
            sleep_time.tv_nsec = desired_loop_rate_.expectedCycleTime().toNSec() - elapsed_time.toNSec();
            if (sleep_time.tv_nsec > 10000)
            {
                sleep_time.tv_nsec -= 10000; // remove some overhead from sleep calculation
                int rc;
                while ((rc=nanosleep(&sleep_time, &remaining_time)) == EINTR)
                {
                    sleep_time = remaining_time;
                }
                // Debug tracking
                if (rc != 0)
                {
//                    ros::Time sleep_end = ros::Time::now();
//                    ros::Duration slept_time = sleep_end - end_time;
//                    ROS_INFO(" sleep time in controller update sleep was tspec= %ld.%ld elapsed=%ld slept=%ld rc=%d", sleep_time.tv_sec,  sleep_time.tv_nsec,elapsed_time.toNSec(),slept_time.toNSec(),rc);
//                    ROS_INFO(" remaining time in controller update sleep tspec= %ld.%ld", remaining_time.tv_sec,remaining_time.tv_nsec);
                    ++sleep_failure_; // track statistics
                }
            }
            else if (sleep_time.tv_nsec < 0)
            { // Debug tracking of loop timing issues
                //ROS_INFO(" sleep time in controller update sleep tspec= %ld.%ld  elapsed=%ld", sleep_time.tv_sec,  sleep_time.tv_nsec,elapsed_time.toNSec());
                ++sleep_failure_;
            }
        }
    }
    ROS_INFO("Stopped controller %s run loop !",name_.c_str());
}

// Initialization functions
int32_t VigirHumanoidController::initialize(boost::shared_ptr<ros::NodeHandle>& mode_control_nh,
                                            boost::shared_ptr<ros::NodeHandle>& robot_control_nh,
                                            boost::shared_ptr<ros::NodeHandle>& pub_nh,
                                            boost::shared_ptr<ros::NodeHandle>& sub_nh,
                                            boost::shared_ptr<ros::NodeHandle>& private_nh)
{
    mode_controller_nh_      = mode_control_nh;
    robot_controller_nh_     = robot_control_nh;
    pub_nh_                  = pub_nh    ;
    sub_nh_                  = sub_nh    ;
    private_nh_              = private_nh;

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
            error_status("Robot controllers failed to initialize",rc);
            return ROBOT_CONTROLLERS_FAILED_TO_INITIALIZE;
        }

        // Assign pointer to list of the active controllers
        active_controllers_list_ = robot_hw_interface_->getActiveControllersList();

    }
    catch(...) // @todo: catch specific exceptions and report
    {
        error_status("Robot controllers failed to initialize (exception)",ROBOT_EXCEPTION_CONTROLLERS_FAILED_TO_INITIALIZE);
        return ROBOT_EXCEPTION_CONTROLLERS_FAILED_TO_INITIALIZE;
    }

    try{
        rc = init_robot_subscribers();
        if (rc)
        {
            error_status("Robot subscribers failed to initialize",rc);
            return ROBOT_SUBSCRIBERS_FAILED_TO_INITIALIZE;
        }
    }
    catch(...) // @todo: catch specific exceptions and report
    {
        error_status("Robot behaviors failed to initialize (exception)",ROBOT_EXCEPTION_SUBSCRIBERS_FAILED_TO_INITIALIZE);
        return ROBOT_EXCEPTION_SUBSCRIBERS_FAILED_TO_INITIALIZE;
    }


}



int32_t VigirHumanoidController::cleanup()
{
    std::cout << " Cleanup the controller ..." << std::endl;
    int32_t rc;
    try{
        std::cout << "      Cleanup the suscribers ..." << std::endl;
        rc =   cleanup_robot_subscribers();

        if (rc)
        {
            cleanup_status("Robot subscribers failed to cleanup properly",rc);
            return ROBOT_SUBSCRIBERS_FAILED_TO_CLEANUP_PROPERLY;
        }
    }
    catch(...) // @todo: catch specific exceptions and report
    {
        cleanup_status("Robot subscribers failed to cleanup properly (exception)");
        return ROBOT_EXCEPTION_SUBSCRIBERS_FAILED_TO_CLEANUP_PROPERLY;
    }

    try{
        std::cout << "      Cleanup the publishers ..." << std::endl;
        rc =   cleanup_robot_publishers();

        if (rc)
        {
            cleanup_status("Robot subscribers failed to cleanup properly",rc);
            return ROBOT_PUBLISHERS_FAILED_TO_CLEANUP_PROPERLY;
        }
    }
    catch(...) // @todo: catch specific exceptions and report
    {
        cleanup_status("Robot subscribers failed to cleanup properly (exception)");
        return ROBOT_EXCEPTION_PUBLISHERS_FAILED_TO_CLEANUP_PROPERLY;
    }

    try{
        std::cout << "      Cleanup the controllers ..." << std::endl;
        rc =   cleanup_robot_controllers();

        if (rc)
        {
            cleanup_status("Robot interface failed to cleanup properly",rc);
            return ROBOT_INTERFACE_FAILED_TO_CLEANUP_PROPERLY;
        }
    }
    catch(...) // @todo: catch specific exceptions and report
    {
        cleanup_status("Robot interface failed to cleanup properly (exception)");
        return ROBOT_EXCEPTION_INTERFACE_FAILED_TO_CLEANUP_PROPERLY;
    }

    try{
        std::cout << "      Cleanup the interface ..." << std::endl;
        rc =   cleanup_robot_interface();
        if (rc)
        {
            cleanup_status("Robot interface failed to cleanup properly",rc);
            return ROBOT_INTERFACE_FAILED_TO_CLEANUP_PROPERLY;
        }
    }
    catch(...) // @todo: catch specific exceptions and report
    {
        cleanup_status("Robot interface failed to cleanup properly (exception)");
        return ROBOT_EXCEPTION_INTERFACE_FAILED_TO_CLEANUP_PROPERLY;
    }
    std::cout << " Done cleanup !" << std::endl;

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

void VigirHumanoidController::cleanup_status(const std::string& msg, int32_t rc)
{ // Assume ROS no longer works during cleanup
    if (rc < 0)
    {
        std::cerr << msg << std::endl;
        std::cout << msg << std::endl;

    }
    else
    {
        std::cerr << msg << "(rc=" << rc << ")" << std::endl;
        std::cout << msg << "(rc=" << rc << ")" <<  std::endl;
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

bool has_element(const std::vector<std::string> & sub, const std::vector<std::string>* large) {  if(sub.size() == 0)return false;   else     return std::includes(large->begin(), large->end(), sub.begin(), sub.end());}

// Process list of old and new controllers to determine unique elements that should be started or stopped, and common elements that may continue to ru
// precondition: assumes that *_controllers lists are sorted vectors of strings
void VigirHumanoidController::processControllerLists(const std::vector<std::string> * const old_controllers,
                            const std::vector<std::string> * const new_controllers,
                            std::vector<std::string> & stop_list, std::vector<std::string> & start_list)
{
    stop_list.clear();
    start_list.clear();
    std::vector<std::string> common_list; // debug

    // print strings from input lists, and 3 output lists to screen for debugging
    size_t old_i = 0;
    size_t new_i = 0;

    while(old_i < old_controllers->size() && new_i < new_controllers->size())
    {
        if(old_controllers->at(old_i).compare(new_controllers->at(new_i)) == 0)
        {
            if(!(common_list.size() > 0 && common_list[common_list.size()-1].compare(old_controllers->at(old_i)) == 0))
            {
                common_list.push_back(old_controllers->at(old_i));
            }

            old_i++;
            new_i++;
        }
        else if(old_controllers->at(old_i).compare(new_controllers->at(new_i)) > 0)
        {
            do
            {
                if(!(new_i != 0  && new_controllers->at(new_i).compare(new_controllers->at(new_i-1)) == 0))
                {
                    start_list.push_back(new_controllers->at(new_i));
                }
                new_i++;
            }
            while(new_i < new_controllers->size() && old_controllers->at(old_i).compare(new_controllers->at(new_i)) > 0);
        }
        else
        {
            do
            {
                if(!(old_i != 0  && old_controllers->at(old_i).compare(old_controllers->at(old_i-1)) == 0))
                {
                    stop_list.push_back(old_controllers->at(old_i));

                }
                old_i++;
            }
            while(old_i < old_controllers->size() && new_controllers->at(new_i).compare(old_controllers->at(old_i)) > 0);
        }
    }

    while(old_i < old_controllers->size())
    {
        if(!(old_i != 0  && old_controllers->at(old_i).compare(old_controllers->at(old_i-1)) == 0))
            stop_list.push_back(old_controllers->at(old_i));
        old_i++;
    }
    while(new_i < new_controllers->size())
    {
        if(!(new_i != 0  && new_controllers->at(new_i).compare(new_controllers->at(new_i-1)) == 0))
            start_list.push_back(new_controllers->at(new_i));
        new_i++;
    }


    //// For now simple stop all, start all (will not play nice with footstep controller!)
    //stop_list = *old_controllers;
    //start_list = *new_controllers;

    //vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv DEBUG vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
    // @todo - remove this debug block
    //The following checks if the common list is included in all the elements and that stop_list and start_list both are not contained in new_controllers and old_controllers respectively

    using namespace std;
    if(has_element(stop_list, new_controllers) || has_element(start_list, old_controllers)//Comment this out if you would like everything to be printed out
            || !std::includes(new_controllers->begin(), new_controllers->end(), common_list.begin(), common_list.end())
            || !std::includes(old_controllers->begin(), old_controllers->end(), common_list.begin(), common_list.end())
            || (common_list.size() > 0) && (std::includes(start_list.begin(), start_list.end(), common_list.begin(), common_list.end())
                                        ||  std::includes(stop_list.begin(),  stop_list.end(),  common_list.begin(), common_list.end())))
    {
            cout << "ERROR FOUND: " ;
            cout << has_element(stop_list, new_controllers) << " " << has_element(start_list, old_controllers) << " ";
            cout << !std::includes(new_controllers->begin(), new_controllers->end(), common_list.begin(), common_list.end()) << " " ;
            cout << !std::includes(old_controllers->begin(), old_controllers->end(), common_list.begin(), common_list.end());
            cout << endl;

        cout << "Old controllers: " <<  *old_controllers << endl;
        cout << "New controllers: " <<  *new_controllers << endl;

    }
    std::cout << "Control Stop  List: " << stop_list   << std::endl;
    std::cout << "        Start List: " << start_list  << std::endl;
    std::cout << "       Common List: " << common_list << std::endl;
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
}

} /* namespace flor_control */
