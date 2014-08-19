
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


class TestHumanoidInterface : public VigirHumanoidInterface
{
  public:

    TestHumanoidInterface(const std::string& name,
                          boost::shared_ptr<ros::NodeHandle>& pub_nh,
                          boost::shared_ptr<ros::NodeHandle>& private_nh) :
        VigirHumanoidInterface(name, pub_nh, private_nh) {
        ROS_INFO("Construct the HumanoidInterface");
    }

    ~TestHumanoidInterface() {
        ROS_ERROR("Destroy the HumanoidInterface");
    }


    int32_t initialize_interface()
    {
        ROS_ERROR("Need to finish initializing the robot interface");
        return ROBOT_INTERFACE_OK;
    }

    int32_t cleanup_interface()
    {
        ROS_ERROR("Need to finish shutting down the robot interface");
        return ROBOT_INTERFACE_OK;
    }

    void update_state_data()     // from robot
    {
       ROS_INFO("dummy update_state_data()");
    }

    void publish_state_data()     // from robot
    {
       ROS_INFO("dummy publish state data()");
    }

    void update_behavior_data()
    {
        ROS_INFO("dummy update_behavior_data()");

    }

    void publish_behavior_data()     // from robot
    {
       ROS_INFO("dummy publish state data()");
    }

   void send_controller_data()// to robot
   {
       ROS_INFO("dummy send_controller_data()");
   }

   void send_behavior_data()  // to   robot
   {
    ROS_INFO("dummy send_behavior_data()");
   }

};

class TestHumanoidHWInterface : public VigirHumanoidHWInterface
{
  public:

    TestHumanoidHWInterface(const std::string& name) :
        VigirHumanoidHWInterface(name) {
        ROS_INFO("Construct the HumanoidInterface");
    }

    ~TestHumanoidHWInterface() {
        ROS_ERROR("Destroy the HumanoidInterface");
    }


    int32_t init_robot_model()
    {
        // Implementation specific types
        robot_model_.reset(new VigirRobotRBDLModel());

        // Load robot model from parameters
        std::cout << "Reading xml file from parameter server" << std::endl;
        std::string urdf_xml, full_urdf_xml,xml_result;
        urdf_xml = std::string("robot_description");
        if (!private_nh_->searchParam(urdf_xml,full_urdf_xml))
        {
            std::cerr << "Failed to find robot_description on parameter server!" << std::endl;
            return 1;
        }

        if (!private_nh_->getParam(full_urdf_xml, xml_result))
        {
            std::cerr << "Failed to load the robot urdf from parameter server for test!" << std::endl;
            return 1;
        }
        //std::cout << " URDF:\n" << xml_result << std::endl;

        // Load joint list from parameters
        if (!private_nh_->searchParam("controlled_joints",full_urdf_xml))
        {
            std::cerr << "Failed to find controlled joints list on parameter server!" << std::endl;
            return 1;
        }

        std::vector<std::string> controlled_joints;
        if (!private_nh_->getParam(full_urdf_xml, controlled_joints))
        {
            std::cerr << "Failed to load the controlled joint list for test!" << std::endl;
            return 1;
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

        // Set up data to hold controller data
        this->current_robot_state_.reset(new VigirRobotStateData(robot_model_->n_joints_));
        this->current_robot_behavior_.reset(new VigirRobotBehaviorData());

        // Initialize the robot interface
        robot_interface_.reset(new TestHumanoidInterface(name_,pub_nh_, private_nh_));
        robot_interface_->initialize_states(robot_model_->n_joints_);


        return ROBOT_INITIALIZED_OK;
    }

    int32_t init_robot_filters()
    {
        boost::shared_ptr<vigir_control::VigirRobotFilterBase>     joint_filter; // filter type chosen by implementation
        boost::shared_ptr<vigir_control::VigirRobotPoseFilterBase> pose_filter;  // filter type chosen by implementation

        joint_filter.reset(new vigir_control::VigirRobotBasic2StateKF(name_+"/joint_filter",robot_model_->n_joints_));
        pose_filter.reset( new vigir_control::VigirRobotPoseFilter(name_+"/pose_filter"));

        robot_interface_->initialize_filters(joint_filter, pose_filter);
        ROS_ERROR("Need to finish initializing the robot filters with paramters");

        return ROBOT_INITIALIZED_OK;
    }

    int32_t init_robot_calibration()
    {
        ROS_ERROR("Need to finish initializing the robot calibration");
        robot_interface_->robot_calibration_.reset(new VigirRobotCalibrationBase());

        return ROBOT_INITIALIZED_OK;
    }


    int32_t init_robot_interface()
    {
        ROS_ERROR("Need to finish initializing the actual robot API interface");
        return ROBOT_INITIALIZED_OK;
    }

    int32_t cleanup_robot_interface()
    {
        ROS_ERROR("Need to finish shutting down the robot interface");
        return ROBOT_CLEANUP_OK;
    }

    int32_t cleanup_robot_model()
    {
        ROS_ERROR("Need to finish shutting down the robot interface");
        return ROBOT_CLEANUP_OK;
    }

    void read_state_data()     // from robot
    {
       ROS_INFO("dummy read_state_data()");
       current_robot_state_ = robot_interface_->filtered_robot_state_;
    }

   void write_controller_data()// to robot
   {
       ROS_INFO("dummy send_controller_data()");
       robot_interface_->send_controller_data();
   }
   void read_behavior_data( )     // to  controllers
   {
        ROS_INFO("dummy read_behavior_data()");
        current_robot_behavior_ = robot_interface_->current_robot_behavior_;
   }
   void write_behavior_data( ) // from controllers
   {
        ROS_INFO("dummy write_behavior_data()");
        robot_interface_->send_behavior_data();
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

    int32_t cleanup() { };

    int32_t initialize(boost::shared_ptr<ros::NodeHandle>& beh_nh,
                       boost::shared_ptr<ros::NodeHandle>& control_nh,
                       boost::shared_ptr<ros::NodeHandle>& pub_nh,
                       boost::shared_ptr<ros::NodeHandle>& private_nh)
    {

        robot_hw_interface_.reset(new TestHumanoidHWInterface(name_));
        robot_hw_interface_->initialize(beh_nh, control_nh, pub_nh, private_nh);
    }

  private:
    // Implementation specific functions

};

}
#include <ros/ros.h>
int main(int argc, char ** argv)
{
    ros::init(argc,argv,"");

    std::cout <<"\n\n\nStart robot model test" << std::endl;

    boost::shared_ptr<ros::NodeHandle> main_nh(new ros::NodeHandle());
    boost::shared_ptr<ros::NodeHandle> nhp(new ros::NodeHandle("~"));

    { // Scope to test destructor before final exit
        vigir_control::TestHumanoidController test_controller("Test");

        test_controller.initialize(main_nh,main_nh,main_nh,nhp);

        ROS_INFO("Need to do something here to test");
    }


    std::cout << "Done!" << std::endl;

   return 0;
}
