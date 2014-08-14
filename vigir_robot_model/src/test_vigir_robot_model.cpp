
//#include <flor_dynamics/FlorStability.h>
#include <vigir_robot_model/VigirRobotRBDLModel.h>
#include <vigir_robot_model/VigirRobotState.h>
#include <fstream>
#include <flor_utilities/timing.h>

#include <ros/ros.h>
int main(int argc, char ** argv)
{
    ros::init(argc,argv,"");

    std::cout <<"\n\n\nStart robot model test" << std::endl;

//    std::ifstream in("~/atlas.urdf");
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

    if (uint32_t rc = robot_model.loadRobotModel(xml_result, 2.0,true))
    {
        printf("Failed to load the robot model (rc=%d)- abort!\n",rc);
        return 1;
    }
    else
    {
        printf("Successfully loaded the robot URDF model!\n");
    }


    Timing calc_torque_timing_("VigirRobotRBDLModel:: Required torque calc",true,false);
    Timing com_calc_timing_            ("VigirRobotRBDLModel:: CoM Calc",true,false);
    Timing update_kinematics_timing_   ("VigirRobotRBDLModel:: Update kinematics",true,false);
    Timing calc_ee_transforms_timing_   ("VigirRobotRBDLModel:: Calc Transforms",true,false);
    vigir_control::VigirRobotState robot_state(robot_model.n_joints_);
    vigir_control::VectorNd torques = robot_state.current_robot_state_.robot_joints_.joint_positions_;
    vigir_control::PoseZYX pelvis_pose;
    vigir_control::Vector3d CoM;
    double mass;
    printf(" Start loop of 1000 calls to model calculations ...\n");
   for (int i = 0; i < 1000; ++i)
   {
    printf("\r counter=%d",i);
    pelvis_pose.position[0] =  0.0;
    pelvis_pose.position[1] =  0.0;
    pelvis_pose.position[2] =  0.0;

    pelvis_pose.orientation[0] =  0.0;
    pelvis_pose.orientation[1] =  0.0;
    pelvis_pose.orientation[2] =  0.0;

    {DO_TIMING(update_kinematics_timing_)
        robot_model.updateJointState(1234,
                                     robot_state.current_robot_state_.robot_joints_.joint_positions_,
                                     robot_state.current_robot_state_.robot_joints_.joint_velocities_,
                                     robot_state.current_robot_state_.robot_joints_.joint_accelerations_);
        robot_model.updateKinematics( );
    }
    {DO_TIMING(calc_torque_timing_)
        robot_model.calcRequiredTorques();
    }
    { DO_TIMING(com_calc_timing_)
        robot_model.calcCOM();
    }
    {DO_TIMING(calc_ee_transforms_timing_)
        robot_model.calcEETransforms();
    }
    robot_model.getRequiredTorques(torques);
    robot_model.getCoM(CoM,mass);
    //std::cout << "  mass=" << mass << " CoM = " << CoM.transpose() << std::endl;
    //std::cout << "  T=" << torques.transpose() << std::endl;

    //std::cout << "Shift pelvis" << std::endl;
    pelvis_pose.position[0] =  2.5;
    pelvis_pose.position[1] = -1.0;
    pelvis_pose.position[2] =  1.0;

    pelvis_pose.orientation[0] =  0.0;
    pelvis_pose.orientation[1] =  0.2;
    pelvis_pose.orientation[2] =  0.0;

    robot_state.current_robot_state_.robot_joints_.joint_positions_[1] = 0.5;

    robot_model.updateBasePose(pelvis_pose);
    //std::cout << "Update joint states " << std::endl;
    {DO_TIMING(update_kinematics_timing_)
        robot_model.updateJointState(1234,
                                     robot_state.current_robot_state_.robot_joints_.joint_positions_,
                                     robot_state.current_robot_state_.robot_joints_.joint_velocities_,
                                     robot_state.current_robot_state_.robot_joints_.joint_accelerations_);
        robot_model.updateKinematics( );
    }
    {DO_TIMING(calc_torque_timing_)
        robot_model.calcRequiredTorques();
    }
    { DO_TIMING(com_calc_timing_)
        robot_model.calcCOM();
    }
    {DO_TIMING(calc_ee_transforms_timing_)
        robot_model.calcEETransforms();
    }
    robot_model.getRequiredTorques(torques);
    //std::cout << "  mass=" << mass << " CoM = " << CoM.transpose() << std::endl;
    //std::cout << "  T=" << torques.transpose() << std::endl;
  }
    std::cout << std::endl;
    std::cout << "Done!" << std::endl << std::endl << std::endl << std::endl;
    return 0;
}