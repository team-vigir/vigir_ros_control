
//#include <flor_dynamics/FlorStability.h>
#include <vigir_humanoid_controller/VigirHumanoidController.h>

class TestHumanoidController: public VigirHumanoidController
{

    TestHumanoidController(const std::string& name)
        : VigirHumanoidController(name)
    {
        ROS_INFO("Initialize TestHumanoidController");
    }

    ~VigirHumanoidController()
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
    int32_t init_robot_interface()
    {
        ROS_WARNING("Dummy init_robot_interface");
        return ROBOT_INITIALIZED_OK;
    }

    int32_t cleanup_robot_interface()
    {
        ROS_WARNING("Dummy cleanup_robot_interface");

        return ROBOT_CLEANUP_OK;
    }

};

#include <ros/ros.h>
int main(int argc, char ** argv)
{
    ros::init(argc,argv,"");

    std::cout <<"\n\n\nStart robot model test" << std::endl;

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    { // Scope to test destructor before final exit
        TestHumanoidController test_controller("Test");

        test_controller.initialize();

        ROS_INFO("Need to do something here to test");
    }


    std::cout << "Done!" << std::endl;

   return 0;
}
