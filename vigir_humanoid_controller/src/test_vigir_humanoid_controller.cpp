
#include <vigir_humanoid_controller/VigirHumanoidController.h>

#include<vigir_robot_model/VigirRobotRBDLModel.h>
#include<vigir_robot_model/VigirRobotBasic2StateKF.h>
#include<vigir_robot_model/VigirRobotPoseFilter.h>


namespace vigir_control
{

class TestHumanoidInterface : public VigirHumanoidInterface
{
  public:

    TestHumanoidInterface(const std::string& name) :
        VigirHumanoidInterface(name) {
        ROS_INFO("Construct the HumanoidInterface");
    }

    ~TestHumanoidInterface() {
        ROS_ERROR("Destroy the HumanoidInterface");
    }

    int32_t initialize_models()
    {
        // Implementation specific types
        robot_model_.reset(new VigirRobotRBDLModel());

        // Load joint list from parameters

        // Load robot model from parameters

        // Load model from URDF

        // Setup joints

        joint_filter_.reset(new VigirRobotBasic2StateKF(name_+"/joint_filter",robot_model_->n_joints_));
        pose_filter_.reset(new VigirRobotPoseFilter(name_+"/pose_filter"));

        // state vectors
        ROS_ERROR("Finish initialization with state structures");
        return VigirHumanoidController::ROBOT_INITIALIZED_OK;
    }

    int32_t initialize_interface()
    {
        ROS_ERROR("Need to finish intializing the robot interface");
        return VigirHumanoidController::ROBOT_INITIALIZED_OK;
    }

    void update_state_data()     // from robot
    {
       ROS_INFO("dummy update_state_data()");
    }

   void send_controller_data()// to robot
   {
       ROS_INFO("dummy send_controller_data()");
   }
   void read_state_data(vigir_control::VigirRobotStateData& )     // to  controllers
   {
    ROS_INFO("dummy read_state_data()");
   }
   void write_controller_data(const vigir_control::VigirRobotStateData& ) // from controllers
   {
    ROS_INFO("dummy write_controller_data()");
   }

   void update_behavior_data()// from robot
   {
    ROS_INFO("dummy update_behavior_data()");
   }
   void send_behavior_data()  // to   robot
   {
    ROS_INFO("dummy send_behavior_data()");
   }
   void read_behavior_data(VigirRobotBehaviorData&)         // to   controllers
   {
    ROS_INFO("dummy read_behavior_data()");
   }
   void write_behavior_data(const VigirRobotBehaviorData& ) // from controllers
   {
    ROS_INFO("dummy write_behavior_data()");
   }

};


class TestHumanoidController : public VigirHumanoidController
{
  public:
    TestHumanoidController(const std::string& name)
        : VigirHumanoidController(name)
    {
        ROS_INFO("Initialize TestHumanoidController");
    }

    ~TestHumanoidController()
    {
        ROS_INFO("Destroy TestHumanoidController");
        cleanup();
        ROS_INFO("Done destruction of TestHumanoidController");

    }

    void read(ros::Time time, ros::Duration period)
    {
        ROS_INFO("Read controller input data");
    }

    void write(ros::Time time, ros::Duration period)
    {
        ROS_INFO("Read controller input data");
    }

  private:
    // Implementation specific functions
    int32_t init_robot_model()
    {
        ROS_WARN("Test init_robot_model");
        try {
            robot_interface_.reset(new vigir_control::TestHumanoidInterface( this->name_ ) );
            robot_interface_->initialize_models();
        }
        catch(...)
        {
            return ROBOT_MODEL_FAILED_TO_INITIALIZE;
        }

        return ROBOT_INITIALIZED_OK;

    }

    int32_t init_robot_interface()
    {
        ROS_WARN("Dummy init_robot_interface");
        return ROBOT_INITIALIZED_OK;
    }

    int32_t cleanup_robot_interface()
    {
        ROS_WARN("Dummy cleanup_robot_interface");

        return ROBOT_CLEANUP_OK;
    }

};

}
#include <ros/ros.h>
int main(int argc, char ** argv)
{
    ros::init(argc,argv,"");

    std::cout <<"\n\n\nStart robot model test" << std::endl;

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    { // Scope to test destructor before final exit
        vigir_control::TestHumanoidController test_controller("Test");

        test_controller.initialize();

        ROS_INFO("Need to do something here to test");
    }


    std::cout << "Done!" << std::endl;

   return 0;
}
