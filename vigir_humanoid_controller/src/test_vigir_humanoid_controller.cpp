/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//@TODO_ADD_AUTHOR_INFO

#include <vigir_humanoid_controller/VigirHumanoidController.h>

#include<vigir_robot_model/VigirRobotRBDLModel.h>
#include<vigir_robot_model/VigirRobotBasic2StateKF.h>
#include<vigir_robot_model/VigirRobotPoseFilter.h>
#include <vigir_humanoid_controller/VigirRealTimeBuffer.h>
#include <vigir_humanoid_controller/VigirRobotCalibration.h>
#include <vigir_robot_model/VigirRobotModel.h>
#include <vigir_robot_model/VigirRobotState.h>
#include <vigir_humanoid_controller/VigirHumanoidStatusCodes.h>

#include <std_msgs/String.h>
#include <vigir_humanoid_controller/VigirRealTimePublisher.h>

namespace vigir_control
{

class TestHumanoidHWInterface : public VigirHumanoidHWInterface
{
public:
    TestHumanoidHWInterface(const std::string& name) :  VigirHumanoidHWInterface(name) {}

    std::vector< std::string > test;

    // Define pure virtual functions
    const int32_t                       getActiveControlModeId() { return -1;}
    const std::vector< std::string >*   getActiveJointControllersList(){return &test;}
    const std::vector< std::string >*   getActiveRobotControllersList(){return &test;}
    const VigirHumanoidSwitchMode       permitControllerSwitch()  {return SWITCH_IMMEDIATE;}
};

class VigirRobotBehaviorData  // temporary dummy class definition
{
public:
    VigirRobotBehaviorData() {};
};


// Single structure that can be passed between threads
typedef struct VigirRobotInterfaceData
{
    VigirRobotInterfaceData(const int32_t& n_joints = 0)
        : current_robot_state_(n_joints),filtered_robot_state_(n_joints)
    {}

    vigir_control::VigirRobotStateData      current_robot_state_;    // structure to store latest robot state data
    vigir_control::VigirRobotStateData      filtered_robot_state_;   // structure to store filtered robot state data

    vigir_control::VigirRobotBehaviorData   current_robot_behavior_;
} VigirRobotInterfaceData;

typedef struct VigirRobotControlData
{
    VigirRobotControlData(const int32_t& n_joints = 0)
        : robot_joints_(n_joints) {}

    // Internal representations of control data
    uint64_t                        last_update_time_;

    Pose                            desired_pelvis_pose_;
    VigirRobotJointData             robot_joints_;
    VigirRobotBehaviorData          desired_robot_behavior_;
    // @todo footstep plan

} VigirRobotControlData;

class TestHumanoidInterface : public VigirHumanoidInterface
{
  public:

    // Construct the interface and initialize the state joint sizes
    TestHumanoidInterface(const std::string& name, const int32_t& n_joints) :
        VigirHumanoidInterface(name, n_joints),
        robot_state_(vigir_control::VigirRobotInterfaceData(n_joints), "InterfaceData"),
        robot_control_(vigir_control::VigirRobotControlData(n_joints), "ControlData")
    {
        ROS_INFO("Construct the HumanoidInterface");
    }

    ~TestHumanoidInterface() {
        ROS_INFO("Destroy the HumanoidInterface");
    }


    int32_t initialize_interface()
    {
        ROS_INFO("Need to finish initializing the robot interface");

        // set up the robot API

        // start any worker threads
        return ROBOT_INITIALIZED_OK;
    }

    int32_t cleanup_interface()
    {
        ROS_INFO("Need to finish shutting down the robot interface");

        // shutdown threads

        // free resources
        return ROBOT_CLEANUP_OK;
    }

    void update_state_data()    {} // from robot
    void send_controller_data() {} // to robot

    void update_behavior_data()  {} // from robot
    void send_behavior_data()    {} // to robot

   // Data for passing between controllers and robot interface
   // Must be thread safe for this particular setup
   VigirRealTimeBuffer<vigir_control::VigirRobotInterfaceData>  robot_state_;     // structure to store latest robot state data
   VigirRealTimeBuffer<vigir_control::VigirRobotControlData>    robot_control_;   // structure to store filtered robot state data

};

class TestHumanoidController : public VigirHumanoidController
{
  public:

    // Construct specific interfaces
    TestHumanoidController(const std::string& name, const ros::Rate& loop_rate = ros::Rate(500))
        : VigirHumanoidController(name,loop_rate)
    {
        ROS_INFO("Initialize TestHumanoidController");
        robot_model_.reset(new VigirRobotRBDLModel()); // define model type here
    }

    ~TestHumanoidController()
    {
        ROS_INFO("Destroy TestHumanoidController");
        ROS_INFO("Done destruction of TestHumanoidController");


    }

  private:

    // Implementation specific choices for robot interface
    int32_t init_robot_interface()
    {

        robot_interface_.reset(new TestHumanoidInterface(name_,robot_model_->n_joints_));

        // Set up the filters
        robot_interface_->joint_filter_.reset(new vigir_control::VigirRobotBasic2StateKF(name_+"/joint_filter",robot_model_->n_joints_));
        robot_interface_->pose_filter_.reset( new vigir_control::VigirRobotPoseFilter(name_+"/pose_filter"));

        ROS_ERROR("Need to finish initializing the robot filters with paramters");

        // Set up the calibration system
        robot_interface_->robot_calibration_.reset(new VigirRobotCalibration(name_+"/calibration",robot_model_->n_joints_));

        ROS_ERROR("Need to finish initializing the robot calibration with paramters");

        // Now initialize the specific interface
        return robot_interface_->initialize_interface();
    }

    int32_t init_robot_controllers()
    {
        ROS_INFO("  initialize robot controllers from derived Controller");

        // Initialize the robot hardware interface here to allow override
        robot_hw_interface_.reset(new TestHumanoidHWInterface(name_));

        // Set up the controller manager and assign a specific call backand controller manager
        mode_cm_.reset(new controller_manager::ControllerManager(robot_hw_interface_.get(), *mode_controller_nh_.get()));
        robot_cm_.reset(new controller_manager::ControllerManager(robot_hw_interface_.get(), *robot_controller_nh_.get()));

        // Initialize the controllers
        int32_t rc = robot_hw_interface_->init_robot_controllers(robot_model_, mode_controller_nh_, robot_controller_nh_, private_nh_);
        if (rc)
        {
            ROS_ERROR("Failed to initialize the HW interface for controllers - abort!");
            return rc;
        }


    }

    int32_t cleanup_robot_controllers()
    {
        ROS_INFO("  cleanup robot controllers from derived Controller");
        return robot_hw_interface_->cleanup_robot_controllers( );
    }

    int32_t cleanup_robot_interface()
    {
        ROS_INFO("  cleanup robot interface from derived Controller");
        return robot_interface_->cleanup_interface();
    }

    int32_t init_robot_publishers()
    {
        ROS_INFO("init controller pub");
        return ROBOT_INITIALIZED_OK;
    }

    int32_t cleanup_robot_publishers()
    {
        ROS_INFO("cleanup controller pub");
        return ROBOT_CLEANUP_OK;
    }

    int32_t init_robot_subscribers()
    {
        ROS_INFO("init controller pub");
        return ROBOT_INITIALIZED_OK;
    }

    int32_t cleanup_robot_subscribers()
    {
        ROS_INFO("cleanup controller pub");
        return ROBOT_CLEANUP_OK;
    }


    bool read(ros::Time time, ros::Duration period)
    {
        ROS_INFO("Read - controller");
        return true;
    }

    void write(ros::Time time, ros::Duration period)
    {
        ROS_INFO("Write - controller");

    }

    bool newDataAvailable(){return true;}

};

}
#include <ros/ros.h>
int main(int argc, char ** argv)
{
    ros::init(argc,argv,"");

    vigir_control::VigirRobotStateData data_test(3);
    data_test.last_update_time_ = 20L;

    std::cout << "Start RTB test " << std::endl;
    vigir_control::VigirRealTimeBuffer<vigir_control::VigirRobotStateData> state_data(data_test, "RobotStateData");

    vigir_control::VigirRobotStateData read;
    state_data.readBuffer(read);
    if (read.last_update_time_ != data_test.last_update_time_)
    {
        std::cerr << "invalid buffer reference" << std::endl;
        std::cerr <<  read.last_update_time_ << " : " << data_test.last_update_time_ << std::endl;
    }

    data_test.last_update_time_ = 22;
    state_data.getWritableReference() = data_test;
    state_data.setReadBuffer();
    state_data.readBuffer(read);

    if (read.last_update_time_ != data_test.last_update_time_)
    {
        std::cerr << "invalid buffer const reference" << std::endl;
        std::cerr <<  read.last_update_time_ << " : " << data_test.last_update_time_ << std::endl;
    }

    data_test.last_update_time_ = 23;
    *(state_data.getWriteablePtr()) = data_test;
    state_data.setReadBuffer();

    state_data.readBuffer(read);
    if (read.last_update_time_ != data_test.last_update_time_)
    {
        std::cerr << "invalid buffer pointer" << std::endl;
        std::cerr <<  read.last_update_time_ << " : " << data_test.last_update_time_ << std::endl;
    }

    data_test.last_update_time_ = 30L;
    state_data.writeBuffer(data_test);
    state_data.readBuffer(read);
    if (read.last_update_time_ != data_test.last_update_time_)
    {
        std::cerr << "invalid buffer reference after write" << std::endl;
        std::cerr <<  read.last_update_time_ << " : " << data_test.last_update_time_ << std::endl;
    }

    std::cout << "Done read buffer simple test" << std::endl;

    //exit(-1);

    std::cout <<"\n\n\nStart real time publisher test" << std::endl;

    boost::shared_ptr<ros::NodeHandle> main_nh(new ros::NodeHandle());
    boost::shared_ptr<ros::NodeHandle> nhp(new ros::NodeHandle("~"));


    // Test RealTimePublisher
    class VigirRobotInterfaceDataTopic : public vigir_control::VigirRealTimeTopicBase<vigir_control::VigirRobotInterfaceData>
    {

    public:
        VigirRobotInterfaceDataTopic(const std::string& topic, ros::NodeHandle& nh,
                                     const ros::Rate& rate = ros::Rate(0), const bool& on_change = false,
                                     uint32_t queue_size = 10, const bool& latch = false)
            : vigir_control::VigirRealTimeTopicBase<vigir_control::VigirRobotInterfaceData>(topic, rate, on_change)//, // must call base class constructor separately
//              vigir_control::VigirRealTimeTopic<std_msgs::String>(topic, nh, rate , on_change ,queue_size, latch)
        {
            ROS_INFO("      Advertising real-time topic <%s>",topic_name_.c_str());
            publisher_ = nh.advertise<std_msgs::String>(topic_name_,queue_size, latch);
        }

        ~VigirRobotInterfaceDataTopic() {}

        bool publish(const vigir_control::VigirRobotInterfaceData& data, const ros::Time& current_time)
        {
            try {
                if (current_time > next_publish_time_)
                {
                    //vigir_control::VigirRobotInterfaceData vigir_data = boost::any_cast<vigir_control::VigirRobotInterfaceData>(data);
                    std_msgs::String msg;
                    msg.data = "> Test "+topic_name_ +" " + boost::lexical_cast<std::string>(current_time.toNSec()) +" " + boost::lexical_cast<std::string>(data.current_robot_state_.last_update_time_);
                    publisher_.publish(msg);
                    next_publish_time_ = current_time + publish_rate_.expectedCycleTime();
                    std::cout << topic_name_ << " " << current_time << "  next=" << next_publish_time_ << std::endl;
                    return true;
                }
             }
            catch(...)
            {
                ROS_ERROR("Bad conversion for VigirRobotInterfaceData");
                return false;
            }
            return false;
        }


    };

    boost::shared_ptr< vigir_control::VigirRealTimeBuffer<vigir_control::VigirRobotInterfaceData> > interface_data;
    interface_data.reset(new vigir_control::VigirRealTimeBuffer<vigir_control::VigirRobotInterfaceData>(vigir_control::VigirRobotInterfaceData(10),"InterfaceData"));

    vigir_control::VigirRealTimePublisher<vigir_control::VigirRobotInterfaceData> rt_pub("Test publisher", interface_data);
    boost::shared_ptr<vigir_control::VigirRealTimeTopicBase<vigir_control::VigirRobotInterfaceData> > topic1( new VigirRobotInterfaceDataTopic("test1",*main_nh.get(),ros::Rate(10),true,100));
    boost::shared_ptr<vigir_control::VigirRealTimeTopicBase<vigir_control::VigirRobotInterfaceData> > topic2( new VigirRobotInterfaceDataTopic("test2",*main_nh.get(),ros::Rate( 1),true,100));
    rt_pub.addTopic(topic1);
    rt_pub.addTopic(topic2);

    ros::spinOnce();
    int count = 0;
    vigir_control::VigirRobotInterfaceData data;

    boost::shared_ptr<boost::thread>  publisher_worker_thread_;
    publisher_worker_thread_.reset( new boost::thread(boost::bind(&vigir_control::VigirRealTimePublisher<vigir_control::VigirRobotInterfaceData>::publishLoop, &rt_pub)));

    while(ros::ok() && count < 150000)
    {
        // Get current data from real time thread
        interface_data->getWriteablePtr()->current_robot_state_.last_update_time_ = ros::Time::now().toNSec();
        interface_data->setReadBuffer();

        //interface_data->readBuffer(data);
        //
        //// Publish all topics
        //rt_pub.publish(data,ros::Time::now());

        // Loop
        usleep(50);
        ++count;
        ros::spinOnce();
    }

    ROS_INFO("Shutdown the publisher thread ...");
    rt_pub.shutdown();
    publisher_worker_thread_->join();
    ROS_INFO("Done!");



    std::cout <<"\n\n\nStart robot model test" << std::endl;
    { // Scope to test destructor before final exit

        // Set up the controller to try and run at 1kHz
        vigir_control::TestHumanoidController test_controller("Test", 1000);

        if (int32_t rc = test_controller.initialize(main_nh,main_nh,main_nh,main_nh,main_nh,nhp))
        {
            ROS_ERROR("Failed to initialize the controller with rc=%d - abort!", rc);
            exit(rc);
        }

        // Run the loop until ROS or other tells the controller to quit
        test_controller.runController();


        ROS_INFO("Explicitly call cleanup before exit!");
        test_controller.cleanup();
    }


    std::cout << "Done!" << std::endl;

   return 0;
}
