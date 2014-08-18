
#include <vigir_humanoid_controller/VigirHumanoidController.h>

#include<vigir_robot_model/VigirRobotRBDLModel.h>
#include<vigir_robot_model/VigirRobotBasic2StateKF.h>
#include<vigir_robot_model/VigirRobotPoseFilter.h>

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

namespace vigir_control
{

class VigirRobotBehaviorData  // temporary dummy class definition
{
public:
    VigirRobotBehaviorData() {};
};

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

        ros::NodeHandle nh;
        ros::NodeHandle nhp("~");

        // Load robot model from parameters
        std::cout << "Reading xml file from parameter server" << std::endl;
        std::string urdf_xml, full_urdf_xml,xml_result;
        urdf_xml = std::string("robot_description");
        if (!nhp.searchParam(urdf_xml,full_urdf_xml))
        {
            std::cerr << "Failed to find robot_description on parameter server!" << std::endl;
            return 1;
        }

        if (!nhp.getParam(full_urdf_xml, xml_result))
        {
            std::cerr << "Failed to load the robot urdf from parameter server for test!" << std::endl;
            return 1;
        }
        //std::cout << " URDF:\n" << xml_result << std::endl;

        // Load joint list from parameters
        if (!nhp.searchParam("controlled_joints",full_urdf_xml))
        {
            std::cerr << "Failed to find controlled joints list on parameter server!" << std::endl;
            return 1;
        }

        std::vector<std::string> controlled_joints;
        if (!nhp.getParam(full_urdf_xml, controlled_joints))
        {
            std::cerr << "Failed to load the controlled joint list for test!" << std::endl;
            return 1;
        }
        std::cout << " Controlled joints:\n" << controlled_joints << std::endl;

        // Load model from URDF
        int32_t rc;
        if (rc = robot_model_->initializeRobotJoints(controlled_joints,
                                                   "pelvis",
                                                   "l_foot", "r_foot",
                                                   "l_hand", "r_hand"))
        {
            std::cerr << "Robot model initialization failed" << std::endl;
            return rc;
        }

        if (uint32_t rc = robot_model_->loadRobotModel(xml_result, 1.0,true))
        {
            printf("Failed to load the robot model (rc=%d)- abort!\n",rc);
            return 1;
        }
        else
        {
            printf("Successfully loaded the robot URDF model!\n");
        }

        // Setup state vectors based on joint list size
        current_robot_state_.reset(   new VigirRobotStateData(robot_model_->n_joints_));     ; // structure to store latest robot state data
        filtered_robot_state_.reset(  new VigirRobotStateData(robot_model_->n_joints_)); // structure to store filtered robot state data
        controlled_robot_state_.reset(new VigirRobotStateData(robot_model_->n_joints_)); // structure to store robot control commands

        current_robot_behavior_.reset(new VigirRobotBehaviorData());
        desired_robot_behavior_.reset(new VigirRobotBehaviorData());

        joint_filter_.reset(new VigirRobotBasic2StateKF(name_+"/joint_filter",robot_model_->n_joints_));
        pose_filter_.reset(new VigirRobotPoseFilter(name_+"/pose_filter"));

        return VigirHumanoidController::ROBOT_INITIALIZED_OK;
    }

    int32_t initialize_interface()
    {
        ROS_ERROR("Need to finish initializing the robot interface");
        return VigirHumanoidController::ROBOT_INITIALIZED_OK;
    }

    int32_t shutdown_interface()
    {
        ROS_ERROR("Need to finish shutting down the robot interface");
        return VigirHumanoidController::ROBOT_CLEANUP_OK;
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
        if (robot_interface_)
        {
            return robot_interface_->initialize_interface();
        }
        else
        {
            return ROBOT_INTERFACE_FAILED_TO_INITIALIZE;
        }
    }

    int32_t cleanup_robot_interface()
    {
        ROS_WARN("Dummy cleanup_robot_interface");
        if (robot_interface_)
        {
            try {
                int32_t rc = robot_interface_->shutdown_interface();
                if (rc)
                {
                    error_status("Failed to shutdown interface properly",rc);
                    return ROBOT_INTERFACE_FAILED_TO_CLEANUP_PROPERLY;

                }
            }
            catch(...)
            {
                return ROBOT_INTERFACE_FAILED_TO_CLEANUP_PROPERLY;

            }
        }

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
