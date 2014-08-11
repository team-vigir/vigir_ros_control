
//#include <flor_dynamics/FlorStability.h>
#include <vigir_robot_model/VigirRobotRBDLModel.h>
#include <vigir_robot_model/VigirRobotState.h>
#include <fstream>

#include <ros/ros.h>
int main(int argc, char ** argv)
{
    ros::init(argc,argv,"");

    std::cout <<"\n\n\nStart robot model test" << std::endl;

//    //std::ifstream in("/home/david/flor_repo/rosbuild_ws/vigir_robot/flor_atlas_description/urdf/atlas.urdf");
//    std::ifstream in("/home/david/atlas.urdf");
//    if (!in.is_open())
//    {
//        std::cerr << "Failed to open the robot urdf file for test!" << std::endl;
//        return 1;
//    }

//    std::stringstream ss;
//    ss << in.rdbuf();

//    std::string xml_result = ss.str();
//    if (xml_result.length() == 0)
//    {
//        std::cerr << "Failed to load the robot urdf file for test!" << std::endl;
//        return 1;
//    }
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    std::string urdf_xml, full_urdf_xml,xml_result;
    urdf_xml = std::string("robot_description");
    // Get URDF XML
    nhp.searchParam(urdf_xml,full_urdf_xml);

    std::cout << "Reading xml file from parameter server" << std::endl;
    if (!nhp.getParam(full_urdf_xml, xml_result))
    {
        std::cerr << "Failed to load the robot urdf from parameter server for test!" << std::endl;
        return 1;
    }
    //std::cout << xml << std::endl << std::endl << std::endl << std::endl << std::endl;

    std::vector<std::string> controlled_joints;
    controlled_joints.clear();;
    controlled_joints.push_back("back_bkz" );
    controlled_joints.push_back("back_bky" );
    controlled_joints.push_back("back_bkx" );
    controlled_joints.push_back("neck_ry"  );
    controlled_joints.push_back("l_leg_hpz");
    controlled_joints.push_back("l_leg_hpx");
    controlled_joints.push_back("l_leg_hpy");
    controlled_joints.push_back("l_leg_kny");
    controlled_joints.push_back("l_leg_aky");
    controlled_joints.push_back("l_leg_akx");
    controlled_joints.push_back("r_leg_hpz");
    controlled_joints.push_back("r_leg_hpx");
    controlled_joints.push_back("r_leg_hpy");
    controlled_joints.push_back("r_leg_kny");
    controlled_joints.push_back("r_leg_aky");
    controlled_joints.push_back("r_leg_akx");
    controlled_joints.push_back("l_arm_shy");
    controlled_joints.push_back("l_arm_shx");
    controlled_joints.push_back("l_arm_ely");
    controlled_joints.push_back("l_arm_elx");
    controlled_joints.push_back("l_arm_wry");
    controlled_joints.push_back("l_arm_wrx");
    controlled_joints.push_back("r_arm_shy");
    controlled_joints.push_back("r_arm_shx");
    controlled_joints.push_back("r_arm_ely");
    controlled_joints.push_back("r_arm_elx");
    controlled_joints.push_back("r_arm_wry");
    controlled_joints.push_back("r_arm_wrx");


    std::cout << " Loading robot model ..." << std::endl;
    vigir_control::VigirRobotRBDLModel      robot_model;
    int rc;
    if (rc = robot_model.initializeRobotJoints(controlled_joints,
                                               "pelvis",
                                               "l_foot", "r_foot",
                                               "l_hand", "r_hand"))
    {
        std::cerr << "Robot model initialization failed" << std::endl;
        return rc;
    }

    if (robot_model.loadRobotModel(xml_result, 1.0,false))
    {
        printf("Successfully loaded the robot URDF model");
    }
    else
    {
        printf("Failed to load the robot model - abort!\n");
        return 1;
    }

    vigir_control::VigirRobotState robot_state(robot_model.n_joints_ = 0);

    robot_model.updateJointState(1234,
                                 robot_state.current_robot_state_.robot_joints_.joint_positions_,
                                 robot_state.current_robot_state_.robot_joints_.joint_velocities_,
                                 robot_state.current_robot_state_.robot_joints_.joint_accelerations_);
    robot_model.updateKinematics(robot_state.current_robot_state_.pelvis_pose_.orientation);

    robot_model.calcCOM();
    robot_model.calcEETransforms();

    vigir_control::Vector3d CoM;
    double mass;
    robot_model.getCoM(CoM,mass);
    std::cout << " CoM = " << CoM << "  mass=" << mass << std::endl;

    std::cout << "Done!" << std::endl << std::endl << std::endl << std::endl;
    return 0;
}
