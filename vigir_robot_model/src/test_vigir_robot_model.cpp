
//#include <flor_dynamics/FlorStability.h>
#include <vigir_robot_model/VigirRobotRBDLModel.h>
#include <fstream>

#include <ros/ros.h>
int main(int argc, char ** argv)
{
    ros::init(argc,argv,"");

    vigir_control::VigirRobotRBDLModel      robot_model;

    std::ifstream in("/usr/share/drcsim-2.2/gazebo_models/atlas_description/atlas_sandia_hands/atlas_sandia_hands.urdf");
    std::stringstream ss;
    ss << in.rdbuf();

    std::string xml = ss.str();

    // Default masses of the base link
    const double mass=17.882;
    const vigir_control::Vector3d com(0.0111, 0.0, 0.0271);
    vigir_control::Matrix3d inertia;
    inertia(0,0)=  0.1244;inertia(0,1)= 0.0008;inertia(0,2)= -0.0007;
    inertia(1,0)=  0.0008;inertia(1,1)= 0.0958;inertia(1,2)= -0.0005;
    inertia(2,0)= -0.0007;inertia(2,1)=-0.0005;inertia(2,2)=  0.1167;

    if (robot_model.loadRobotModel(xml, mass, com, inertia,
                                   "pelvis",
                                   "r_foot", "l_foot",
                                   "r_hand", "l_hand",
                                   "hokuyo",
                                   1.0))
    {
        printf("load = true");
        return 1;
    }
    else
    {
        printf("Successfully loaded the robot URDF model");
    }

    return 0;
}
