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
#include <urdf/model.h>

typedef boost::shared_ptr<urdf::Link> LinkPtr;
typedef boost::shared_ptr<urdf::Joint> JointPtr;

typedef std::vector<LinkPtr> URDFLinkVector;
typedef std::vector<JointPtr> URDFJointVector;
typedef std::map<std::string, LinkPtr > URDFLinkMap;
typedef std::map<std::string, JointPtr > URDFJointMap;

// load original construct_model function rbdl_urdfreader.cc
// @todo - make rbdl_urdfreader a Catkin package to avoid this hack
#include <rbdl/../../addons/urdfreader/rbdl_urdfreader.cc>

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

    double                            mass_;
    RigidBodyDynamics::Math::Vector3d CoM_;


};

VigirRobotRBDLModel::VigirRobotRBDLModel()
    : my_rbdl_(new VigirRobotRBDLPrivate), // define RBDL specific data
      VigirRobotModel() // initialize the base class
{


}

 VigirRobotRBDLModel::~VigirRobotRBDLModel() {
     std::cout << "Destroy the VigirRobotRBDLModel!" << std::endl;
 }

 // Create a RBDL model from URDF xml string
 uint32_t VigirRobotRBDLModel::loadRobotModel(const std::string xml,
                                              const double& mass_factor,
                                              const bool& verbose)
 {

     // We are assuming humanoid form factor and looking for four end effector names
     std::cout << " Initialize robot with:" << std::endl;
     std::cout << lhand_link_name_ << "  " << rhand_link_name_  << std::endl;
     std::cout << "  " << root_link_name_  << std::endl;
     std::cout << lfoot_link_name_ << "  " << rfoot_link_name_  << std::endl;

     std::cout << " Calling urdf model initString ..." << std::endl;
     boost::shared_ptr<urdf::Model> urdf_model_ptr(new urdf::Model());
     bool rc = urdf_model_ptr->initString(xml);
     if (!rc) {
         std::cout << "Failed to load robot model in urdf" << std::endl;
         std::cerr << "Error: Failed to load robot model in urdf" << std::endl;
         return 1;
     }

     std::cout << " Pre-process URDF links by applying mass factor and checking for non-zero rotations in link body" << std::endl;
     URDFLinkMap urdf_link_map;
     urdf_link_map = urdf_model_ptr->links_;
     double total_mass = 0.0;
     int count  = 0;
     vigir_control::Vector3d rpy;
     for(URDFLinkMap::const_iterator lm_itr = urdf_link_map.begin();
         lm_itr != urdf_link_map.end(); ++lm_itr)
     {
         if (lm_itr->second->inertial)
         {
             //printf("Link(% 3d) %s original mass=%f \n",count, lm_itr->second->name.c_str(), lm_itr->second->inertial->mass);
             lm_itr->second->inertial->mass *= mass_factor;
             total_mass += lm_itr->second->inertial->mass;

             lm_itr->second->inertial->origin.rotation.getRPY (rpy[0], rpy[1], rpy[2]);
             if (rpy != vigir_control::Vector3d(0.0, 0.0, 0.0))
             {
                 std::cerr << "Error while processing body '" << lm_itr->second->name << "': rotation of body frames " << rpy.transpose() << " not yet supported. Please rotate the joint frame instead. " << std::endl;
                 if (lm_itr->second->inertial->mass > 1e-4)
                 {
                     std::cerr << "     Significant mass=" << lm_itr->second->inertial->mass << " - but override rotation anyway!" << std::endl;
                 }
                 lm_itr->second->inertial->origin.rotation.setFromRPY(0.0,0.0,0.0);
             }

         }
         //else
         //{
         //    printf("Link(% 3d) %s Non-inertial \n",count, lm_itr->second->name.c_str());
         //}
         ++count;
     }
     printf(" Total links =%d adjusted total mass = %f\n\n",count, total_mass);

     std::cout << " Pre-process URDF joints by FIXing non-controlled joints" << std::endl;
     URDFJointMap urdf_joint_map;
     urdf_joint_map = urdf_model_ptr->joints_;
     for(URDFJointMap::const_iterator jm_itr = urdf_joint_map.begin();
         jm_itr != urdf_joint_map.end(); ++jm_itr)
     {
         try {
             joint_map_.at(jm_itr->first); // looking for existance in map
         }
         catch(...)
         {
             if (jm_itr->second->type != urdf::Joint::FIXED)
             {
                printf("Set uncontrolled joint %s (%s) to FIXED type (%d) instead of %d\n",
                    jm_itr->first.c_str(),
                    jm_itr->second->name.c_str(),
                    urdf::Joint::FIXED,
                    jm_itr->second->type);
                jm_itr->second->type = urdf::Joint::FIXED;
             }
         }
     }

     std::cout << "Construct the RBDL model from the adjusted urdf model ..." << std::endl;
     if (!RigidBodyDynamics::Addons::construct_model(&(my_rbdl_->rbdl_model_), urdf_model_ptr, verbose))
    {
        std::cout << "Failed to load robot model into RBDL format !"<< std::endl;
        std::cerr << "Failed to load robot model into RBDL format !"<< std::endl;
        return 2;
     }
     std::cout << "Done RBDL model construction!" << std::endl;


     printf(" List of all robot body parts (%d fixed, %d movable, %d DOF, %d joints):\n",
            uint8_t(my_rbdl_->rbdl_model_.mFixedBodies.size()), uint8_t(my_rbdl_->rbdl_model_.mBodies.size()),
            my_rbdl_->rbdl_model_.dof_count,uint8_t(my_rbdl_->rbdl_model_.mJoints.size()));
     printf("Note: fixed bodies are already joined to moveable parents\n");
     double total_mass2= 0.0;
     for (int ndx = 0; ndx < int8_t(my_rbdl_->rbdl_model_.mBodies.size()); ++ndx) {
         if ( rfoot_link_name_ == my_rbdl_->rbdl_model_.GetBodyName(ndx))
         {
             r_foot_id_ = ndx;
             printf(" Found end effector %s at %d\n", my_rbdl_->rbdl_model_.GetBodyName(ndx).c_str(),ndx);
         }
         else if ( lfoot_link_name_ == my_rbdl_->rbdl_model_.GetBodyName(ndx))
         {
             l_foot_id_ = ndx;
             printf(" Found end effector %s at %d\n", my_rbdl_->rbdl_model_.GetBodyName(ndx).c_str(),ndx);
         }
         else if ( rhand_link_name_ == my_rbdl_->rbdl_model_.GetBodyName(ndx))
         {
             r_hand_id_ = ndx;
             printf(" Found end effector %s at %d\n", my_rbdl_->rbdl_model_.GetBodyName(ndx).c_str(),ndx);
         }
         else if ( lhand_link_name_ == my_rbdl_->rbdl_model_.GetBodyName(ndx))
         {
             l_hand_id_ = ndx;
             printf(" Found end effector %s at %d\n", my_rbdl_->rbdl_model_.GetBodyName(ndx).c_str(),ndx);
         }

         total_mass2 += my_rbdl_->rbdl_model_.mBodies[ndx].mMass;
         printf("Body  : % 3d :%s : mass=%f\n",
                ndx, my_rbdl_->rbdl_model_.GetBodyName(ndx).c_str(),
                my_rbdl_->rbdl_model_.mBodies[ndx].mMass);
     }
     printf(" Total mass=%f = %f\n\n",total_mass, total_mass2);
     if (( r_foot_id_ < 1) || ( l_foot_id_ < 1) || (r_hand_id_ < 1) || (l_hand_id_ < 1))
     {
         printf(" Failed to find all end effectors feet(%d, %d) hands (%d, %d)\n", l_foot_id_, r_foot_id_, l_hand_id_, r_hand_id_);
         return 3;
     }

     // Maps from one to another representation
     rbdl_to_ctrl_joint.resize(my_rbdl_->rbdl_model_.mJoints.size(),-1);
     ctrl_to_rbdl_joint.resize(n_joints_);
     printf(" %10s : %10s :%s :%s\n", "Joint Name", "Child Link", "Control", "RBDL");

     for(std::map<std::string, int8_t>::const_iterator jm_itr = joint_map_.begin();
         jm_itr != joint_map_.end(); ++jm_itr)
     {
         std::string jnt_name = jm_itr->first;   // controlled joint name
         int8_t      jnt_ndx  = jm_itr->second;  // index into controlled joint list

         try {
            JointPtr urdf_joint = urdf_joint_map.at(jnt_name); // URDF map
            int body_ndx = my_rbdl_->rbdl_model_.GetBodyId(urdf_joint->child_link_name.c_str());
            if (my_rbdl_->rbdl_model_.IsBodyId(body_ndx))
            {
                rbdl_to_ctrl_joint[body_ndx] = jnt_ndx;    // maps body index to the controlled joint that moves the body
                ctrl_to_rbdl_joint[jnt_ndx]  = body_ndx-1; // maps controlled joint to the Q vector index (not what documentation says!)
                printf(" %10s : %10s : % 3d : % 3d\n",
                       jnt_name.c_str(), urdf_joint->child_link_name.c_str(),
                       jnt_ndx, body_ndx);
            }
            else
            {
                printf("Could not find body for joint <%s> - child <%s>",jnt_name.c_str(), urdf_joint->child_link_name.c_str());
                return 4;
            }
         }
         catch(...)
         {
             std::cout << "Could not find controlled joint <" << jnt_name << " in URDF " << std::endl;
             return 5;
         }

     }
     for (uint ndx = 0; ndx < rbdl_to_ctrl_joint.size(); ++ndx)
     {
         printf("RBDL body: %s ctrl ndx=%d\n",my_rbdl_->rbdl_model_.GetBodyName(ndx).c_str(), rbdl_to_ctrl_joint[ndx]);
     }
     for (uint ndx = 0; ndx < ctrl_to_rbdl_joint.size(); ++ndx)
     {
         printf("Controlled link: %d rbdl ndx=%d\n",ndx, ctrl_to_rbdl_joint[ndx]);
     }



     // Initialize the vectors for storing robot state
     my_rbdl_->Q_     = RigidBodyDynamics::Math::VectorNd::Constant ((size_t)my_rbdl_->rbdl_model_.dof_count,0.0); // contra documentation, Q[0] is first joint
     my_rbdl_->QDot_  = RigidBodyDynamics::Math::VectorNd::Constant ((size_t)my_rbdl_->rbdl_model_.dof_count,0.0);
     my_rbdl_->QDDot_ = RigidBodyDynamics::Math::VectorNd::Constant ((size_t)my_rbdl_->rbdl_model_.dof_count,0.0); // set to zero for now
     my_rbdl_->tau_   = RigidBodyDynamics::Math::VectorNd::Constant ((size_t)my_rbdl_->rbdl_model_.dof_count,0.0); // generalized forces

     std::cout << " Done loading RBDL model with "<< my_rbdl_->rbdl_model_.dof_count << " DOF! "<< std::endl << std::endl;
     return 0;
}

 // This function should be called while externally thread protected
 // Store filtered data for use in current threaded function
void VigirRobotRBDLModel::updateJointState(const uint64_t& timestamp, const VectorNd& joint_positions, const VectorNd& joint_velocities, const VectorNd& joint_accelerations)
{
    b_transforms_up_to_date_ = false; // need to recalculate the transforms
    b_CoM_up_to_date_        = false;
    b_positions_up_to_date_  = false;
    b_kinematics_up_to_date_ = false;
    b_dynamics_up_to_date_   = false;

    this->timestamp_ = timestamp;

    for (uint ndx = 0; ndx < n_joints_; ++ndx)
    {
        int8_t rbdl_ndx            = ctrl_to_rbdl_joint[ndx];
        my_rbdl_->Q_[rbdl_ndx]     = joint_positions[ndx];
        my_rbdl_->QDot_[rbdl_ndx]  = joint_velocities[ndx];
        my_rbdl_->QDDot_[rbdl_ndx] = joint_accelerations[ndx]; // Later
    }
}

/** Update pelvis pose in root frame */
void VigirRobotRBDLModel::updateBasePose(const PoseZYX& pose)
{
    my_rbdl_->Q_[0] = pose.position[0];
    my_rbdl_->Q_[1] = pose.position[1];
    my_rbdl_->Q_[2] = pose.position[2];
    my_rbdl_->Q_[3] = pose.orientation[2];
    my_rbdl_->Q_[4] = pose.orientation[1];
    my_rbdl_->Q_[5] = pose.orientation[0];
}

void VigirRobotRBDLModel::updateBaseVelocity(const PoseZYX& velocity)
{
    my_rbdl_->QDot_[0] = velocity.position[0];
    my_rbdl_->QDot_[1] = velocity.position[1];
    my_rbdl_->QDot_[2] = velocity.position[2];
    my_rbdl_->QDot_[3] = velocity.orientation[2];
    my_rbdl_->QDot_[4] = velocity.orientation[1];
    my_rbdl_->QDot_[5] = velocity.orientation[0];
}

void VigirRobotRBDLModel::updateBasePose(const Pose& pose)
{
    my_rbdl_->Q_[0] = pose.position[0];
    my_rbdl_->Q_[1] = pose.position[1];
    my_rbdl_->Q_[2] = pose.position[2];

    urdf::Rotation rot(pose.orientation.x(),pose.orientation.y(),pose.orientation.z(),pose.orientation.w());
    rot.getRPY(my_rbdl_->Q_[3],my_rbdl_->Q_[4],my_rbdl_->Q_[5]);
    //printf("  update Base pose q=[(%f, %f, %f), %f)  RPY=(%f, %f, %f)\n",
    //         pose.orientation.x(),pose.orientation.y(),pose.orientation.z(),pose.orientation.w(), my_rbdl_->Q_[3],my_rbdl_->Q_[4],my_rbdl_->Q_[5]);
}

// Update model with position and velocity
void VigirRobotRBDLModel::updatePositions()
{
    UpdateKinematicsCustom (my_rbdl_->rbdl_model_, &(my_rbdl_->Q_), NULL, NULL);
    b_positions_up_to_date_  = true;
}
void VigirRobotRBDLModel::updateKinematics()
{
    UpdateKinematicsCustom (my_rbdl_->rbdl_model_, &(my_rbdl_->Q_), &(my_rbdl_->QDot_), NULL);
    b_positions_up_to_date_  = true;
    b_kinematics_up_to_date_ = true;
}

// Update model with position, velocity, and accelerations
void VigirRobotRBDLModel::updateDynamics()
{
    UpdateKinematics(my_rbdl_->rbdl_model_, my_rbdl_->Q_, my_rbdl_->QDot_, my_rbdl_->QDDot_);
    b_positions_up_to_date_  = true;
    b_kinematics_up_to_date_ = true;
    b_dynamics_up_to_date_   = true; // need to recalculate the transforms
}

/* Compute CoM in pelvis frame given current robot pose */
void VigirRobotRBDLModel::calcCOM( )
{
    // Compute CoM using RBDL utilities
    RigidBodyDynamics::Utils::CalcCenterOfMass (my_rbdl_->rbdl_model_,
                                                my_rbdl_->Q_,
                                                my_rbdl_->QDot_,
                                                my_rbdl_->mass_,
                                                my_rbdl_->CoM_,
                                                NULL,
                                                false);

    // Convert to internal storage
    CoM_pelvis_  = Vector3d(my_rbdl_->CoM_.x(),my_rbdl_->CoM_.y(),my_rbdl_->CoM_.z());
    mass_        = my_rbdl_->mass_;

    b_CoM_up_to_date_ = true;

}

void VigirRobotRBDLModel::calcEETransforms()
{ // assumes ID properly found
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

    b_transforms_up_to_date_ = true;
}

void VigirRobotRBDLModel::getJointTransform(const int32_t& ctrl_joint_id, Transform& T)
{
    int8_t rbdl_id            = ctrl_to_rbdl_joint[ctrl_joint_id];
    T.translation = RigidBodyDynamics::CalcBodyToBaseCoordinates (
                            my_rbdl_->rbdl_model_,
                            my_rbdl_->Q_,
                            rbdl_id,
                            RigidBodyDynamics::Math::Vector3d(0.0,0.0,0.0), // origin
                            false);// don't update kinematics here

    T.rotation = RigidBodyDynamics::CalcBodyWorldOrientation (
                            my_rbdl_->rbdl_model_,
                            my_rbdl_->Q_,
                            rbdl_id,
                            false).inverse();// don't update kinematics here
}

void VigirRobotRBDLModel::getBaseTransform(Transform& T)
{
    int8_t rbdl_id            = 6; // assumes floating base
    T.translation = RigidBodyDynamics::CalcBodyToBaseCoordinates (
                            my_rbdl_->rbdl_model_,
                            my_rbdl_->Q_,
                            rbdl_id,
                            RigidBodyDynamics::Math::Vector3d(0.0,0.0,0.0), // origin
                            false);// don't update kinematics here

    T.rotation = RigidBodyDynamics::CalcBodyWorldOrientation (
                            my_rbdl_->rbdl_model_,
                            my_rbdl_->Q_,
                            rbdl_id,
                            false).inverse();// don't update kinematics here
}

// Calculate the torques to hold the current state
//   Assumes that gravity vector is specified in the pelvis frame, and current kinematic joint positions and velocities are up to date
void VigirRobotRBDLModel::calcRequiredTorques()
{
    RigidBodyDynamics::InverseDynamics ((my_rbdl_->rbdl_model_),
                                        (my_rbdl_->Q_),     // inputs
                                        (my_rbdl_->QDot_),  // inputs
                                        (my_rbdl_->QDDot_), // inputs
                                        (my_rbdl_->tau_),   // output                                        // outputs
                                         NULL // optional -- later -- external forces std::vector<Math::SpatialVector> *f_ext =
                                       );
}

void VigirRobotRBDLModel::getRequiredTorques(VectorNd& cmd_efforts)
{
    // Load into the joint control vector
    for (uint ndx = 0; ndx < ctrl_to_rbdl_joint.size(); ++ndx)
    {
        int8_t rbdl_ndx  = ctrl_to_rbdl_joint[ndx];
        cmd_efforts[ndx] = my_rbdl_->tau_[rbdl_ndx];
    }
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
