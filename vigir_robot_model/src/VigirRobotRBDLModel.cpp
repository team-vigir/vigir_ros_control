/*=================================================================================================
// Copyright (c) 2013-2014, David Conner, TORC Robotics
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Team ViGIR or TORC Robotics nor the names of
//       its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================
*/

// These must occur before any other definitions to include required Eigen addons
#include <rbdl/Body.h>
#include <rbdl/rbdl.h>

#include <ostream>
#include <stack>
#include <vigir_robot_model/VigirRobotRBDLModel.h>
#include <flor_utilities/timing.h>

namespace vigir_control {


// This private structure hides the RBDL libraries and headers from other packages
// which simplifies the dependencies
struct VigirRobotRBDLPrivate
{
    VigirRobotRBDLPrivate() { std::cout << "Construct RBDL model " << std::endl;};

    RigidBodyDynamics::Model          rbdl_model_;
    RigidBodyDynamics::Math::VectorNd Q_;
    RigidBodyDynamics::Math::VectorNd QDot_;
    RigidBodyDynamics::Math::VectorNd QDDot_;
    RigidBodyDynamics::Math::VectorNd tau_;

};

VigirRobotRBDLModel::VigirRobotRBDLModel()
    : my_rbdl_(new VigirRobotRBDLPrivate), // define RBDL specific data
      VigirRobotModel() // initialize the base class
{


}

 VigirRobotRBDLModel::~VigirRobotRBDLModel() {
     std::cout << "Destroy the VigirRobotRBDLModel!" << std::endl;
 }
// default mass=17.882
// default com = (0.0111, 0.0, 0.0271);
// default interia = [
//  0.1244, 0.0008, -0.0007,
//  0.0008, 0.0958, -0.0005,
// -0.0007,-0.0005,  0.1167]
 bool VigirRobotRBDLModel::loadRobotModel(const std::string xml,
                                          const double& base_mass,
                                          const Vector3d& base_com,
                                          const Matrix3d& base_inertia,
                                          const std::string& root_name,
                                          const std::string& r_foot_name,
                                          const std::string& l_foot_name,
                                          const std::string& r_hand_name,
                                          const std::string& l_hand_name,
                                          const std::string& head_base_name,
                                          const double& mass_factor)
 {

     std::cout << " loadRobotModel ..." << std::endl;
     root_link_name_ = root_name;
     rfoot_link_name_= r_foot_name;
     lfoot_link_name_= l_foot_name;
     rhand_link_name_= r_hand_name;
     lhand_link_name_= l_hand_name;
     head_base_name_ = head_base_name;

     std::cout << " Initialize robot with:" << std::endl;
     std::cout << "          " << head_base_name_  << std::endl;
     std::cout << lhand_link_name_ << "  " << rhand_link_name_  << std::endl;
     std::cout << "          " << root_link_name_  << std::endl;
     std::cout << lfoot_link_name_ << "  " << rfoot_link_name_  << std::endl;

     std::cout << " initString ..." << std::endl;

     urdf::Model urdf_model;
     bool rc = urdf_model.initString(xml);
     if (!rc) {
         std::cout << "Failed to load robot model in urdf" << std::endl;
         std::cerr << "Failed to load robot model in urdf" << std::endl;
         return false;
     }



     std::cout << "Load the RBDL model from urdf using mass factor = " << mass_factor << std::endl;

     std::cout << "Set intertial properities of floating base link ..." << std::endl;
     my_rbdl_->rbdl_model_.mBodies[0].mMass = base_mass;
     my_rbdl_->rbdl_model_.mBodies[0].mCenterOfMass = RigidBodyDynamics::Math::Vector3d(base_com[0],base_com[1],base_com[2]);

     { // inertia calculations from rbdl Body.h
         using namespace RigidBodyDynamics;
         Math::Vector3d com = my_rbdl_->rbdl_model_.mBodies[0].mCenterOfMass;
         Math::Matrix3d com_cross (
                0.  , -com[2],  com[1],
              com[2],    0.  , -com[0],
             -com[1],  com[0],    0.
             );
         Math::Matrix3d parallel_axis;
         parallel_axis = my_rbdl_->rbdl_model_.mBodies[0].mMass * com_cross * com_cross.transpose();

        my_rbdl_->rbdl_model_.mBodies[0].mInertia = Math::Matrix3d (
                    base_inertia(0,0), base_inertia(0,1), base_inertia(0,2),
                    base_inertia(1,0), base_inertia(1,1), base_inertia(1,2),
                    base_inertia(2,0), base_inertia(2,1), base_inertia(2,2));

         Math::Matrix3d pa (parallel_axis);
         Math::Matrix3d mcc = my_rbdl_->rbdl_model_.mBodies[0].mMass * com_cross;
         Math::Matrix3d mccT = mcc.transpose();

         Math::Matrix3d inertia_O = my_rbdl_->rbdl_model_.mBodies[0].mInertia + pa;

        my_rbdl_->rbdl_model_.mBodies[0].mSpatialInertia.set (
             inertia_O(0,0), inertia_O(0,1), inertia_O(0,2), mcc(0, 0), mcc(0, 1), mcc(0, 2),
             inertia_O(1,0), inertia_O(1,1), inertia_O(1,2), mcc(1, 0), mcc(1, 1), mcc(1, 2),
             inertia_O(2,0), inertia_O(2,1), inertia_O(2,2), mcc(2, 0), mcc(2, 1), mcc(2, 2),
             mccT(0, 0), mccT(0, 1), mccT(0, 2), my_rbdl_->rbdl_model_.mBodies[0].mMass, 0., 0.,
             mccT(1, 0), mccT(1, 1), mccT(1, 2), 0., my_rbdl_->rbdl_model_.mBodies[0].mMass, 0.,
             mccT(2, 0), mccT(2, 1), mccT(2, 2), 0., 0., my_rbdl_->rbdl_model_.mBodies[0].mMass
             );
     } // end rbdl Body.h inertia calculations

     bool verbose = false; // don't dump the RBDL data

     if (!construct_model (&urdf_model, verbose, mass_factor)) {
        std::cout << "Failed to load robot model into RBDL format!"<< std::endl;
        std::cerr << "Failed to load robot model into RBDL format!"<< std::endl;
        return false;
     }

     // Assume model in upright pose
     my_rbdl_->rbdl_model_.gravity.set (0., 0., -9.81);

     my_rbdl_->Q_     = RigidBodyDynamics::Math::VectorNd::Constant ((size_t)my_rbdl_->rbdl_model_.dof_count,0.0); // contra documentation, Q[0] is first joint
     my_rbdl_->QDot_  = RigidBodyDynamics::Math::VectorNd::Constant ((size_t)my_rbdl_->rbdl_model_.dof_count,0.0);
     my_rbdl_->QDDot_ = RigidBodyDynamics::Math::VectorNd::Constant ((size_t)my_rbdl_->rbdl_model_.dof_count,0.0); // set to zero for now
     my_rbdl_->tau_   = RigidBodyDynamics::Math::VectorNd::Constant ((size_t)my_rbdl_->rbdl_model_.dof_count,0.0); // generalized forces

     std::cout << " Done loading RBDL model! " << std::endl;
     return true;
}

 // This function should be called while externally thread protected
 // Store filtered data for use in current threaded function
void VigirRobotRBDLModel::updateJointState(const uint64_t& timestamp, const VectorNd& joint_positions, const VectorNd& joint_velocities, const VectorNd& joint_accelerations)
{

    this->timestamp_ = timestamp;

    for (uint ndx = 0; ndx < n_joints_; ++ndx)
    {
        int8_t rbdl_ndx      = urdf_to_rbdl_joint[ndx];
        my_rbdl_->Q_[rbdl_ndx]     = joint_positions[ndx];
        my_rbdl_->QDot_[rbdl_ndx]  = joint_velocities[ndx];
        my_rbdl_->QDDot_[rbdl_ndx] = joint_accelerations[ndx]; // Later
    }
}

void VigirRobotRBDLModel::updateKinematics(const Quatd& pelvis_orientation)
{
    // Update the gravity vector used by model
    pelvis_orientation_ = pelvis_orientation;
    Quatd qinv = pelvis_orientation_.inverse();

    gravity_pelvis_ = qinv*vigir_control::Vector3d(0.0,0.0,-9.81); // gravity expressed in pelvis frame
    my_rbdl_->rbdl_model_.gravity.set (gravity_pelvis_.x(),gravity_pelvis_.y(),gravity_pelvis_.z());

    UpdateKinematicsCustom (my_rbdl_->rbdl_model_, &(my_rbdl_->Q_), &(my_rbdl_->QDot_), &(my_rbdl_->QDDot_));
    b_transforms_up_to_date = false; // need to recalculate the transforms

}

/* Computer CoM in pelvis frame given current robot pose */
void VigirRobotRBDLModel::calcCOM( )
{
    // Compute CoM using RBDL
    RigidBodyDynamics::Math::Vector3d   base_coords(0.0,0.0,0.0);

    // Get the pelvis data
    double                              rbdl_mass = my_rbdl_->rbdl_model_.mBodies[0].mMass;
    RigidBodyDynamics::Math::Vector3d   rbdl_CoG  = my_rbdl_->rbdl_model_.mBodies[0].mMass*my_rbdl_->rbdl_model_.mBodies[0].mCenterOfMass;

     // Note: Fixed body parameters are already joint to moveable parents
     for (uint ndx = 1; ndx < my_rbdl_->rbdl_model_.mBodies.size(); ++ndx)
     {
         // Transform body CoM into base coordinates
         base_coords = RigidBodyDynamics::CalcBodyToBaseCoordinates (
                 my_rbdl_->rbdl_model_,
                 my_rbdl_->Q_,
                 ndx,
                 my_rbdl_->rbdl_model_.mBodies[ndx].mCenterOfMass, // CoM in body frame
                 false);// don't update kinematics here

         rbdl_mass += my_rbdl_->rbdl_model_.mBodies[ndx].mMass;
         rbdl_CoG  += my_rbdl_->rbdl_model_.mBodies[ndx].mMass * base_coords;

        //     printf("   Body   %d: %s : %f : %f, %f, %f : %f, %f, %f : %f\n",
        //              ndx, my->rbdl_model_.GetBodyName(ndx).c_str(),
        //              my->rbdl_model_.mBodies[ndx].mMass,
        //              base_coords.x(),base_coords.y(),base_coords.z(),
        //              rbdl_CoG.x(), rbdl_CoG.y(), rbdl_CoG.z(),rbdl_mass );
     }

 CoM_  = Vector3d(rbdl_CoG.x(),rbdl_CoG.y(),rbdl_CoG.z());
 CoM_ *= (1.0/rbdl_mass);
 mass_ = rbdl_mass;

}

void VigirRobotRBDLModel::calcEETransforms()
{ // assumes ID properly found
    DO_TIMING( calc_ee_transforms_timing_);

    r_foot_transform_.translation = RigidBodyDynamics::CalcBodyToBaseCoordinates (
                            my_rbdl_->rbdl_model_,
                            my_rbdl_->Q_,
                            r_foot_id_,
                            RigidBodyDynamics::Math::Vector3d(0.0,0.0,0.0), // origin
                            false);// don't update kinematics here

    l_foot_transform_.translation = RigidBodyDynamics::CalcBodyToBaseCoordinates (
                            my_rbdl_->rbdl_model_,
                            my_rbdl_->Q_,
                            l_foot_id_,
                            RigidBodyDynamics::Math::Vector3d(0.0,0.0,0.0), // origin
                            false);// don't update kinematics here

    r_hand_transform_.translation = RigidBodyDynamics::CalcBodyToBaseCoordinates (
                            my_rbdl_->rbdl_model_,
                            my_rbdl_->Q_,
                            r_hand_id_,
                            RigidBodyDynamics::Math::Vector3d(0.0,0.0,0.0), // origin

                            false);// don't update kinematics here

    l_hand_transform_.translation = RigidBodyDynamics::CalcBodyToBaseCoordinates (
                            my_rbdl_->rbdl_model_,
                            my_rbdl_->Q_,
                            l_hand_id_,
                            RigidBodyDynamics::Math::Vector3d(0.0,0.0,0.0), // origin
                            false);// don't update kinematics here

    r_foot_transform_.rotation = RigidBodyDynamics::CalcBodyWorldOrientation (
                            my_rbdl_->rbdl_model_,
                            my_rbdl_->Q_,
                            r_foot_id_,
                            false).inverse();// don't update kinematics here

    l_foot_transform_.rotation = RigidBodyDynamics::CalcBodyWorldOrientation (
                            my_rbdl_->rbdl_model_,
                            my_rbdl_->Q_,
                            l_foot_id_,
                            false).inverse();// don't update kinematics here

    r_hand_transform_.rotation = RigidBodyDynamics::CalcBodyWorldOrientation (
                            my_rbdl_->rbdl_model_,
                            my_rbdl_->Q_,
                            r_hand_id_,
                            false).inverse();// don't update kinematics here

    l_hand_transform_.rotation = RigidBodyDynamics::CalcBodyWorldOrientation (
                            my_rbdl_->rbdl_model_,
                            my_rbdl_->Q_,
                            l_hand_id_,
                            false).inverse();// don't update kinematics here

    b_transforms_up_to_date = true;
}


// Calculate the torques to hold the current state
//   Assumes that gravity vector is specified in the pelvis frame, and current kinematic joint positions and velocities are up to date
void VigirRobotRBDLModel::calcRequiredTorques()
{
    RigidBodyDynamics::InverseDynamics ((my_rbdl_->rbdl_model_),
                                        (my_rbdl_->Q_),
                                        (my_rbdl_->QDot_),
                                        (my_rbdl_->QDDot_), // inputs
                                        (my_rbdl_->tau_),                                           // outputs
                                         NULL // optional -- later -- external forces std::vector<Math::SpatialVector> *f_ext =
                                       );
}

void VigirRobotRBDLModel::getRequiredTorques(VectorNd& cmd_efforts)
{
    // Load into the state vector
    for (uint ndx = 0; ndx < urdf_to_rbdl_joint.size(); ++ndx)
    {
        int8_t rbdl_ndx  = urdf_to_rbdl_joint[ndx];
        cmd_efforts[ndx] = my_rbdl_->tau_[rbdl_ndx];
    }
}


bool VigirRobotRBDLModel::construct_model( urdf::Model *urdf_model, bool verbose, const double& mass_factor) {
    using namespace RigidBodyDynamics;
    using namespace Math;
    using namespace std;

    typedef boost::shared_ptr<urdf::Link> LinkPtr;
    typedef boost::shared_ptr<urdf::Joint> JointPtr;

    typedef std::vector<LinkPtr> URDFLinkVector;
    typedef std::vector<JointPtr> URDFJointVector;
    typedef std::map<std::string, LinkPtr > URDFLinkMap;
    typedef std::map<std::string, JointPtr > URDFJointMap;

    boost::shared_ptr<urdf::Link> urdf_root_link;

    RigidBodyDynamics::Model* rbdl_model = &(my_rbdl_->rbdl_model_);

    URDFLinkMap link_map;
    link_map = urdf_model->links_;

    URDFJointMap joint_map;
    joint_map = urdf_model->joints_;

    vector<string> joint_names;

    stack<LinkPtr > link_stack;
    stack<int> joint_index_stack;

    // add the bodies in a depth-first order of the model tree
    printf(" Initial root link = <%s>\n", urdf_model->getRoot()->name.c_str());
    link_stack.push (link_map[(urdf_model->getRoot()->name)]);

    if (link_stack.top()->child_joints.size() > 0) {
        joint_index_stack.push(0);
    }

    while (link_stack.size() > 0) {
        LinkPtr cur_link = link_stack.top();
        int joint_idx = joint_index_stack.top();

        if (joint_idx < int(cur_link->child_joints.size())) {
            JointPtr cur_joint = cur_link->child_joints[joint_idx];

            // increment joint index
            joint_index_stack.pop();
            joint_index_stack.push (joint_idx + 1);

            link_stack.push (link_map[cur_joint->child_link_name]);
            joint_index_stack.push(0);

            if (verbose) {
                for (int i = 1; i < int(joint_index_stack.size()) - 1; i++) {
                    cout << "  ";
                }
                cout << "joint '" << cur_joint->name << "' child link '" << link_stack.top()->name << " type = " << cur_joint->type << endl;
            }

            joint_names.push_back(cur_joint->name);
        } else {
            link_stack.pop();
            joint_index_stack.pop();
        }
    }

    std::string base_joint = root_link_name_;
    std::cout << "Initial base joint name = " << base_joint << std::endl;
    unsigned int j;
    for (j = 0; j < joint_names.size(); j++) {
        JointPtr urdf_joint = joint_map[joint_names[j]];
        LinkPtr urdf_parent = link_map[urdf_joint->parent_link_name];
        LinkPtr urdf_child = link_map[urdf_joint->child_link_name];

        // determine where to add the current joint and child body
        unsigned int rbdl_parent_id = 0; // initialize to the base joint

        if (urdf_parent->name != base_joint && rbdl_model->mBodies.size() != 1)
            rbdl_parent_id = rbdl_model->GetBodyId (urdf_parent->name.c_str());


        // @todo: This is Atlas specific code due to joint names
        if ((std::string::npos != urdf_child->name.find("left")) ||
            (std::string::npos != urdf_child->name.find("right")) ||
            (std::string::npos != urdf_child->name.find("hokuyo")) )
        {
            printf("Assign child body <%s> as fixed joint ", urdf_child->name.c_str());
            urdf_joint->type = urdf::Joint::FIXED;
        }

        // create the joint
        Joint rbdl_joint;
        if (urdf_joint->type == urdf::Joint::REVOLUTE || urdf_joint->type == urdf::Joint::CONTINUOUS) {
            rbdl_joint = Joint (SpatialVector (urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z, 0., 0., 0.));
        } else if (urdf_joint->type == urdf::Joint::PRISMATIC) {
            rbdl_joint = Joint (SpatialVector (0., 0., 0., urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z));
        } else if (urdf_joint->type == urdf::Joint::FIXED) {
            rbdl_joint = Joint (JointTypeFixed);
        } else if (urdf_joint->type == urdf::Joint::FLOATING) {
            // todo: what order of DoF should be used?
            std::cout << "Processing urdf::Joint::FLOATING for " << urdf_child->name << std::endl;
            rbdl_joint = Joint (
                    SpatialVector (0., 0., 0., 1., 0., 0.),
                    SpatialVector (0., 0., 0., 0., 1., 0.),
                    SpatialVector (0., 0., 0., 0., 0., 1.),
                    SpatialVector (1., 0., 0., 0., 0., 0.),
                    SpatialVector (0., 1., 0., 0., 0., 0.),
                    SpatialVector (0., 0., 1., 0., 0., 0.));
        } else if (urdf_joint->type == urdf::Joint::PLANAR) {
            // todo: which two directions should be used that are perpendicular
            // to the specified axis?
            cerr << "Error while processing joint '" << urdf_joint->name << "': planar joints not yet supported!" << endl;
            return false;
        }

        // compute the joint transformation
        Vector3d joint_rpy;        
        urdf_joint->parent_to_joint_origin_transform.rotation.getRPY (joint_rpy[0], joint_rpy[1], joint_rpy[2]);

        Vector3d joint_translation(
                urdf_joint->parent_to_joint_origin_transform.position.x,
                urdf_joint->parent_to_joint_origin_transform.position.y,
                urdf_joint->parent_to_joint_origin_transform.position.z
                );
        SpatialTransform rbdl_joint_frame =
                  Xrot (joint_rpy[0], Vector3d (1., 0., 0.))
                * Xrot (joint_rpy[1], Vector3d (0., 1., 0.))
                * Xrot (joint_rpy[2], Vector3d (0., 0., 1.))
                * Xtrans (Vector3d (
                            joint_translation
                            ));

        // assemble the body
        Vector3d link_inertial_position = Vector3d (0., 0., 0.);
        Vector3d link_inertial_rpy      = Vector3d (0., 0., 0.);
        Matrix3d link_inertial_inertia  = Matrix3d::Zero();
        double link_inertial_mass       = 0.0;

        // but only if we actually have inertial data
        if (urdf_child->inertial) {
            link_inertial_mass = urdf_child->inertial->mass * mass_factor;

            link_inertial_position = Vector3d(
                    urdf_child->inertial->origin.position.x,
                    urdf_child->inertial->origin.position.y,
                    urdf_child->inertial->origin.position.z
                    );
            urdf_child->inertial->origin.rotation.getRPY (link_inertial_rpy[0], link_inertial_rpy[1], link_inertial_rpy[2]);

            link_inertial_inertia(0,0) = urdf_child->inertial->ixx;
            link_inertial_inertia(0,1) = urdf_child->inertial->ixy;
            link_inertial_inertia(0,2) = urdf_child->inertial->ixz;

            link_inertial_inertia(1,0) = urdf_child->inertial->ixy;
            link_inertial_inertia(1,1) = urdf_child->inertial->iyy;
            link_inertial_inertia(1,2) = urdf_child->inertial->iyz;

            link_inertial_inertia(2,0) = urdf_child->inertial->ixz;
            link_inertial_inertia(2,1) = urdf_child->inertial->iyz;
            link_inertial_inertia(2,2) = urdf_child->inertial->izz;

            if (link_inertial_rpy != Vector3d (0., 0., 0.)) {
                cerr << "Error while processing body '" << urdf_child->name << "': rotation of body frames not yet supported. Please rotate the joint frame instead." << endl;
                cerr << " I=[[ " << link_inertial_inertia(0,0) << ", " << link_inertial_inertia(0,1) << ", " << link_inertial_inertia(0,2) << "]; " << std::endl;
                cerr << "    [ " << link_inertial_inertia(1,0) << ", " << link_inertial_inertia(1,1) << ", " << link_inertial_inertia(1,2) << "]; " << std::endl;
                cerr << "    [ " << link_inertial_inertia(2,0) << ", " << link_inertial_inertia(2,1) << ", " << link_inertial_inertia(2,2) << "]] " << std::endl;
                cerr << "  OK hacking with R*I*R^T - see what that does" << std::endl;
                Quatd   quat;

                urdf_child->inertial->origin.rotation.getQuaternion(quat.x(),quat.y(),quat.z(),quat.w());

                Matrix3d Rm = quat.toRotationMatrix();
                cerr << " R=[[ " << Rm(0,0) << ", " << Rm(0,1) << ", " << Rm(0,2) << "]; " << std::endl;
                cerr << "    [ " << Rm(1,0) << ", " << Rm(1,1) << ", " << Rm(1,2) << "]; " << std::endl;
                cerr << "    [ " << Rm(2,0) << ", " << Rm(2,1) << ", " << Rm(2,2) << "]] " << std::endl;
                Matrix3d Rt = Rm.transpose();
                link_inertial_inertia =Rm*link_inertial_inertia*Rt;
                cerr << " I=[[ " << link_inertial_inertia(0,0) << ", " << link_inertial_inertia(0,1) << ", " << link_inertial_inertia(0,2) << "]; " << std::endl;
                cerr << "    [ " << link_inertial_inertia(1,0) << ", " << link_inertial_inertia(1,1) << ", " << link_inertial_inertia(1,2) << "]; " << std::endl;
                cerr << "    [ " << link_inertial_inertia(2,0) << ", " << link_inertial_inertia(2,1) << ", " << link_inertial_inertia(2,2) << "]] " << std::endl;

            }
        }


        RigidBodyDynamics::Math::Vector3d link_inertial_position_r = link_inertial_position;
        RigidBodyDynamics::Math::Matrix3d link_inertial_inertia_r = link_inertial_inertia;

        Body rbdl_body = Body (link_inertial_mass, link_inertial_position_r, link_inertial_inertia_r);

        if (verbose) {
            cout << "+ Adding Body " << endl;
            cout << "  parent_id  : " << rbdl_parent_id << endl;
            cout << "  joint_frame: " << rbdl_joint_frame << endl;
            cout << "  joint dofs : " << rbdl_joint.mDoFCount << endl;
            for (unsigned int j = 0; j < rbdl_joint.mDoFCount; j++) {
                cout << "    " << j << ": " << rbdl_joint.mJointAxes[j].transpose() << endl;
            }
            cout << "  body inertia: " << endl << rbdl_body.mSpatialInertia << endl;
            cout << "  body_name  : " << urdf_child->name << endl;
        }

        uint body_id = rbdl_model->AddBody (rbdl_parent_id, rbdl_joint_frame, rbdl_joint, rbdl_body, urdf_child->name);
        printf("Add Body : %d : %s  : joint %s : parent %s (%d) : mass= %f \n",
               body_id, urdf_child->name.c_str(),
               urdf_joint->name.c_str(), urdf_parent->name.c_str(), rbdl_parent_id, link_inertial_mass);

        if ( rfoot_link_name_ == urdf_child->name)
        {
            r_foot_id_ = body_id;
            printf(" Found end effector %s at %d\n", urdf_child->name.c_str(),body_id);
        }
        else if ( lfoot_link_name_ == urdf_child->name)
        {
            l_foot_id_ = body_id;
            printf(" Found end effector %s at %d\n", urdf_child->name.c_str(),body_id);
        }
        else if ( rhand_link_name_ == urdf_child->name)
        {
            r_hand_id_ = body_id;
            printf(" Found end effector %s at %d\n", urdf_child->name.c_str(),body_id);
        }
        else if ( lhand_link_name_ == urdf_child->name)
        {
            l_hand_id_ = body_id;
            printf(" Found end effector %s at %d\n", urdf_child->name.c_str(),body_id);
        }
    }

    printf(" List of all robot body parts (%d fixed, %d movable, %d DOF, %d joints):\n",
           uint8_t(rbdl_model->mFixedBodies.size()), uint8_t(rbdl_model->mBodies.size()),
           rbdl_model->dof_count,uint8_t(rbdl_model->mJoints.size()));
    printf("Note: fixed bodies are already joined to moveable parents\n");
    for (int ndx = 0; ndx < int8_t(rbdl_model->mBodies.size()); ++ndx) {
             printf("Body  : % 3d :%s : mass=%f\n",
                    ndx, rbdl_model->GetBodyName(ndx).c_str(),
                    rbdl_model->mBodies[ndx].mMass);
    }

    // Maps from one to another representation
    n_joints_ = rbdl_model->mJoints.size();  // @todo - fix this Atlas?
    rbdl_to_urdf_joint.resize(rbdl_model->mJoints.size(),-1);
    urdf_to_rbdl_joint.resize(n_joints_);
    printf(" %10s : %10s :%s :%s\n", "Joint Name", "Child Link", "OSRF", "RBDL");

    /// HACK ALERT - CHECK OLD NAMES IF NEW NAMES FAIL  // @todo - fix this Atlas?
    std::vector<std::string> old_joint_names;
        old_joint_names.push_back("back_lbz"); old_joint_names.push_back("back_mby"); old_joint_names.push_back("back_ubx");
        old_joint_names.push_back("neck_ay" );
        old_joint_names.push_back("l_leg_uhz"); old_joint_names.push_back( "l_leg_mhx"); old_joint_names.push_back("l_leg_lhy"); old_joint_names.push_back("l_leg_kny" ); old_joint_names.push_back("l_leg_uay" ); old_joint_names.push_back( "l_leg_lax" );
        old_joint_names.push_back("r_leg_uhz"); old_joint_names.push_back( "r_leg_mhx"); old_joint_names.push_back("r_leg_lhy"); old_joint_names.push_back("r_leg_kny" ); old_joint_names.push_back("r_leg_uay" ); old_joint_names.push_back( "r_leg_lax" );
        old_joint_names.push_back("l_arm_usy"); old_joint_names.push_back( "l_arm_shx"); old_joint_names.push_back("l_arm_ely"); old_joint_names.push_back("l_arm_elx" ); old_joint_names.push_back("l_arm_uwy" ); old_joint_names.push_back( "l_arm_mwx" );
        old_joint_names.push_back("r_arm_usy"); old_joint_names.push_back( "r_arm_shx"); old_joint_names.push_back("r_arm_ely"); old_joint_names.push_back("r_arm_elx" ); old_joint_names.push_back("r_arm_uwy" ); old_joint_names.push_back( "r_arm_mwx" );

    for(std::map<std::string, int8_t>::const_iterator jm_itr = joint_map_.begin();
        jm_itr != joint_map_.end(); ++jm_itr)
    {
        std::string jnt_name = jm_itr->first;
        int8_t      jnt_ndx  = jm_itr->second;  // index into osrf array

        JointPtr urdf_joint = joint_map[jnt_name];
        if (!urdf_joint)
        {
            ROS_WARN("Could not find osrf joint <%s> in URDF try old name=%s",jnt_name.c_str(),old_joint_names[jnt_ndx].c_str());
            urdf_joint = joint_map[old_joint_names[jnt_ndx]];
            if (!urdf_joint)
            {
                ROS_ERROR("    Failed again!");
                return false;
            }
        }

        int body_ndx = rbdl_model->GetBodyId(urdf_joint->child_link_name.c_str());
        if (rbdl_model->IsBodyId(body_ndx))
        {
            rbdl_to_urdf_joint[body_ndx] = jnt_ndx; // maps body index to the osrf joint that moves the body
            urdf_to_rbdl_joint[jnt_ndx]  = body_ndx-1; // maps osrf joint to the Q vector index (not what documentation says!)
            printf(" %10s : %10s : % 3d : % 3d\n",
                   jnt_name.c_str(), urdf_joint->child_link_name.c_str(),
                   jnt_ndx, body_ndx);
        }
        else
        {
            ROS_ERROR("Could not find body for joint <%s> - child <%s>",jnt_name.c_str(), urdf_joint->child_link_name.c_str());
            return false;
        }
    }
    for (uint ndx = 0; ndx < rbdl_to_urdf_joint.size(); ++ndx)
    {
        printf("RBDL body: %s osrf ndx=%d\n",rbdl_model->GetBodyName(ndx).c_str(), rbdl_to_urdf_joint[ndx]);
    }
    for (uint ndx = 0; ndx < urdf_to_rbdl_joint.size(); ++ndx)
    {
        printf("OSRF link: %d rbdl ndx=%d\n",ndx, urdf_to_rbdl_joint[ndx]);
    }

    if (( r_foot_id_ < 1) || ( l_foot_id_ < 1) || (r_hand_id_ < 1) || (l_hand_id_ < 1))
    {
        printf(" Failed to find all end effectors feet(%d, %d) hands (%d, %d)\n", l_foot_id_, r_foot_id_, l_hand_id_, r_hand_id_);
        return false;
    }
    return true;
}

void VigirRobotRBDLModel::getLeftHandMass(Vector3d& CoM, float& mass)
{
    CoM  = my_rbdl_->rbdl_model_.mBodies[l_hand_id_].mCenterOfMass, // CoM in body frame
    mass = my_rbdl_->rbdl_model_.mBodies[l_hand_id_].mMass;
}

void VigirRobotRBDLModel::getRightHandMass(Vector3d& CoM, float& mass)
{
    CoM  = my_rbdl_->rbdl_model_.mBodies[r_hand_id_].mCenterOfMass, // CoM in body frame
    mass = my_rbdl_->rbdl_model_.mBodies[r_hand_id_].mMass;
}

void VigirRobotRBDLModel::setLeftHandMass( const Vector3d& CoM, const float& mass, const Vector3d& Ix, const Vector3d& Iy, const Vector3d& Iz)
{
    my_rbdl_->rbdl_model_.mBodies[l_hand_id_].mCenterOfMass = CoM; // CoM in body frame
    my_rbdl_->rbdl_model_.mBodies[l_hand_id_].mMass         = mass;

// @todo - enable this code (and add to right hand) once we are setting the inertia vectors from external
//    { // inertia calculations from rbdl Body.h
//        using namespace RigidBodyDynamics;
//        Math::Vector3d com = my->rbdl_model_.mBodies[l_hand_id_].mCenterOfMass;
//        Math::Matrix3d com_cross (
//                 0., -com[2],  com[1],
//             com[2],      0., -com[0],
//            -com[1],  com[0],      0.
//            );
//        Math::Matrix3d parallel_axis;
//        parallel_axis = my->rbdl_model_.mBodies[l_hand_id_].mMass * com_cross * com_cross.transpose();

//       my->rbdl_model_.mBodies[l_hand_id_].mInertia = Math::Matrix3d (
//                   Ix[0], Iy[0], Iz[0],
//                   Ix[1], Iy[1], Iz[1],
//                   Ix[2], Iy[2], Iz[2] );

//        Math::Matrix3d pa (parallel_axis);
//        Math::Matrix3d mcc = my->rbdl_model_.mBodies[0].mMass * com_cross;
//        Math::Matrix3d mccT = mcc.transpose();

//        Math::Matrix3d inertia_O = my->rbdl_model_.mBodies[0].mInertia + pa;

//       my->rbdl_model_.mBodies[0].mSpatialInertia.set (
//            inertia_O(0,0), inertia_O(0,1), inertia_O(0,2), mcc(0, 0), mcc(0, 1), mcc(0, 2),
//            inertia_O(1,0), inertia_O(1,1), inertia_O(1,2), mcc(1, 0), mcc(1, 1), mcc(1, 2),
//            inertia_O(2,0), inertia_O(2,1), inertia_O(2,2), mcc(2, 0), mcc(2, 1), mcc(2, 2),
//            mccT(0, 0), mccT(0, 1), mccT(0, 2), my->rbdl_model_.mBodies[0].mMass, 0., 0.,
//            mccT(1, 0), mccT(1, 1), mccT(1, 2), 0., my->rbdl_model_.mBodies[0].mMass, 0.,
//            mccT(2, 0), mccT(2, 1), mccT(2, 2), 0., 0., my->rbdl_model_.mBodies[0].mMass
//            );
//    } // end rbdl Body.h inertia calculations

}

void VigirRobotRBDLModel::setRightHandMass(const Vector3d& CoM, const float& mass, const Vector3d& Ix, const Vector3d& Iy, const Vector3d& Iz)
{
    my_rbdl_->rbdl_model_.mBodies[r_hand_id_].mCenterOfMass = CoM; // CoM in body frame
    my_rbdl_->rbdl_model_.mBodies[r_hand_id_].mMass         = mass;
}



} /* namespace vigir_control */
