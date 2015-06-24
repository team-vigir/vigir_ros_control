
#include <vigir_robot_model/VigirRobotPoseFilter.h>
#include <vigir_robot_model/VigirRobotState.h>
#include <fstream>
#include <vigir_utilities/timing.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

typedef boost::normal_distribution<double> NormalDistribution;
typedef boost::mt19937 RandomGenerator;
typedef boost::variate_generator<RandomGenerator&, \
                        NormalDistribution> GaussianGenerator;

 /** Initiate Random Number generator with current time */
 static RandomGenerator rng(static_cast<unsigned> (time(0)));


int main(int argc, char ** argv)
{
    ros::init(argc,argv,"");

    int num_joints = 28;
    int steps      = 5000; // simulate 5 seconds of data
    double dt      = 0.001;
    double elapsed = dt*(steps-1);

    std::cout << "Define test of "  << steps  << " steps at dt="  << dt  << "with " << num_joints << " joints" << std::endl;
    vigir_control::VigirRobotPoseFilter pose_filter("SimpleFilter");
    vigir_control::VigirRobotStateData  robot_state(num_joints);  // state estimate

    /* Choose Normal Distribution and create generator*/
    NormalDistribution gaussian_dist_05(0.0, 0.5);
    GaussianGenerator generator_05(rng, gaussian_dist_05);

    NormalDistribution gaussian_dist_01(0.0, 0.1);
    GaussianGenerator generator_01(rng, gaussian_dist_01);

    NormalDistribution gaussian_dist_001(0.0, 0.01);
    GaussianGenerator generator_001(rng, gaussian_dist_001);

    NormalDistribution gaussian_dist_002(0.0, 0.02);
    GaussianGenerator generator_002(rng, gaussian_dist_002);

    NormalDistribution gaussian_dist_005(0.0, 0.05);
    GaussianGenerator generator_005(rng, gaussian_dist_005);

    NormalDistribution gaussian_dist_15(0.0, 1.5);
    GaussianGenerator generator_15(rng, gaussian_dist_15);

//    calculate_test_data(actual, control, 0.0);
//    sense_test_data(sensed,    actual,  generator_05,  generator_01, generator_01);
//    sense_test_data(control,   control, generator_001, generator_002,generator_002); // bad process control
//    sense_test_data(estimated, sensed,  generator_15 , generator_15, generator_15); // initial with bad data



    // Define vectors to store data for plotting
    std::cout << "Define vectors to store data" << std::endl;
    std::vector<vigir_control::Pose>  actual_poses;
    std::vector<vigir_control::Twist> actual_twists;
    std::vector<vigir_control::Pose>  sensed_poses;
    std::vector<vigir_control::Twist> sensed_twists;
    std::vector<vigir_control::Pose>  estimated_poses;
    std::vector<vigir_control::Twist> estimated_twists;

    // Initialize vector to hold data for plotting results
    std::cout << "Initialize vectors"  << std::endl;
//    actual_positions.resize(steps,    actual.joint_positions_);
//    actual_velocities.resize(steps,   actual.joint_velocities_);
//    sensed_positions.resize(steps,    sensed.joint_positions_);
//    sensed_velocities.resize(steps,   sensed.joint_velocities_);
//    estimated_positions.resize(steps, estimated.joint_positions_);
//    estimated_velocities.resize(steps,estimated.joint_velocities_);


    Timing prediction_timing_("BasicKF:: prediction",true,false);
    Timing correction_timing_("BasicKF:: correction",true,false);
    std::cout << "Begin simulation loop ..."  << std::endl;
    for (int i = 1; i < steps; ++i)
    {
        printf("\r Step %d",i);fflush(stdout);

        // Calculate based on actual dynamics model
        //printf("\n Calc %d",i);fflush(stdout);
        //calculate_test_data(actual, control, i*dt);

        // Sense the actual data with noise
        //printf("\n Sense %d",i);fflush(stdout);
        //sense_test_data(sensed,    actual,  generator_005, generator_01, generator_01);

        // Corrupt the control used for prediction
        //printf("\n Control %d",i);fflush(stdout);
        //sense_test_data(control,   control, generator_001, generator_002, generator_002); // bad process control

        //printf("\n Predict %d",i);fflush(stdout);
        {DO_TIMING(prediction_timing_)
//            basic_kf.predict_filter(estimated.joint_positions_,
//                                    estimated.joint_velocities_,
//                                    control.joint_accelerations_,
//                                    dt);
        }

        ///printf("\n Correct %d",i);fflush(stdout);
        {DO_TIMING(correction_timing_)
//            basic_kf.correct_filter(estimated.joint_positions_,
//                                    estimated.joint_velocities_,
//                                    sensed.joint_positions_,
//                                    sensed.joint_velocities_);
        }
        //printf("\n Loop %d",i);fflush(stdout);

        // Store data for later plotting
//        actual_poses[i]     = actual.joint_positions_;
//        actual_twists[i]    = actual.joint_velocities_;
//        sensed_poses[i]     = sensed.joint_positions_;
//        sensed_twists[i]    = sensed.joint_velocities_;
//        estimated_poses[i]  = estimated.joint_positions_;
//        estimated_twists[i] = estimated.joint_velocities_;
    }

    std::cout << "\nWrite out plot files in python ..." << std::endl;
    //for (int i = 0; i < num_joints; ++i)
    {   // For each joint create a python file to plot the data
        std::stringstream ss;
        ss << 0;//i;
        std::ofstream out;
        std::string filename = "joint_"+ss.str()+".py";
        out.open(filename.c_str(), std::ofstream::out);
        out << "import numpy as np" << std::endl;
        out << "import matplotlib.pyplot as plt" << std::endl;
        out << "t=np.linspace(0, " << elapsed << ", " << steps<< ")" << std::endl << std::endl;
        // Store data in python file
//        store_data(out, "q_act", actual_positions,  i);
//        store_data(out, "dq_act",actual_velocities, i);
//        store_data(out, "q_sensed", sensed_positions,  i);
//        store_data(out, "dq_sensed",sensed_velocities, i);
//        store_data(out, "q_estimated", estimated_positions,  i);
//        store_data(out, "dq_estimated",estimated_velocities, i);

//        // plot the data
//        out << std::endl;
//        out << "plt.subplot(2,1,1)" << std::endl;
//        out << "plt.plot(t,q_act,'g-',label=\"q_act\")" << std::endl;
//        out << "plt.plot(t,q_sensed,'r:',label=\"q_sensed\")" << std::endl;
//        out << "plt.plot(t,q_estimated,'b:',label=\"q_est\")" << std::endl;
//        out << "plt.title(\"Estimation of Joint " << ss.str() <<"\")" << std::endl;
//        out << "plt.ylabel(\"position\")" << std::endl;
//        out << "plt.legend(loc='upper right', shadow=True)" << std::endl;

//        out << "plt.subplot(2,1,2)" << std::endl;
//        out << "plt.plot(t,dq_act,'g-',label=\"dq_act\")" << std::endl;
//        out << "plt.plot(t,dq_sensed,'r:',label=\"dq_sensed\")" << std::endl;
//        out << "plt.plot(t,dq_estimated,'b:',label=\"dq_est\")" << std::endl;
//        out << "plt.title(\"Estimation of Joint " << ss.str() <<"\")" << std::endl;
//        out << "plt.ylabel(\"velocity\")" << std::endl;
//        out << "plt.xlabel(\"time (s)\")" << std::endl;
//        out << "plt.legend(loc='upper right', shadow=True)" << std::endl;
//        out << std::endl;
//        out << "plt.show()" << std::endl;
        out.close();
    }
    std::cout << "\nDone!" << std::endl << std::endl << std::endl << std::endl;
    return 0;
}
