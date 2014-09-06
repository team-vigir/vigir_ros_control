#include <ros/ros.h>
#include <vigir_joint_interfaces/pos_vel_acc_err_humanoid_joint_iface_adapter.h>

struct DummyState
{
std::vector<double> position;
std::vector<double> velocity;
std::vector<double> acceleration;

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_wide_angle");

  ros::NodeHandle nh("pid_settings");
  //std::vector nh_vector;
  //nh_vector.resize(1);
  //nh_vector[0] = ros::NodeHandle("pid_settings");
  
  double measured_pos;
  double measured_vel;
  double measured_eff;

  double commanded_pos;
  double commanded_vel;
  double commanded_acc;
  double error_pos;
  double error_vel;

  hardware_interface::JointStateHandle js_handle("test_joint", &measured_pos, &measured_vel, &measured_eff);
  hardware_interface::PosVelAccErrHumanoidJointHandle handle (js_handle, &commanded_pos, &commanded_vel, &commanded_acc, &error_pos, &error_vel);

  std::vector<hardware_interface::PosVelAccErrHumanoidJointHandle> handle_vector;
  handle_vector.push_back(handle);


  HardwareInterfaceAdapter<hardware_interface::PosVelAccErrHumanoidJointInterface, DummyState> hw_interface_adapter;
  hw_interface_adapter.init(handle_vector, nh);

  int start = -10;
  int end   =  10;
  ros::Time time = ros::Time::now();
  ros::Duration period = ros::Duration(0.001);

  DummyState desired_state;

  // Enter desired state here
  desired_state.position.push_back(1.0);
  desired_state.velocity.push_back(0.0);
  desired_state.acceleration.push_back(0.0);

  DummyState error_state;
  error_state.position.push_back(0.0);
  error_state.velocity.push_back(0.0);
  error_state.acceleration.push_back(0.0);

  for (int i = start; i < end; ++i){

    //Specify measured state here:
    measured_pos = 1.0 + 0.1*i;
    measured_vel = 0.0;

    //Copied from JointTrajectoryController
    error_state.position[0] = desired_state.position[0] - measured_pos;
    error_state.velocity[0] = desired_state.velocity[0] - measured_vel;
    error_state.acceleration[0] = 0.0;

    hw_interface_adapter.updateCommand(time, period, desired_state, error_state);

    time += period;

    std::cout << "mp: " << measured_pos << " mv: " << measured_vel <<"\n";
    std::cout << "dp: " <<  desired_state.position[0] << " dv: " << desired_state.velocity[0] << "\n";
    std::cout << "ca: " << commanded_acc << "\n";
    std::cout << "-----------------------------------------------\n";
  }
  



  return 0;
}
