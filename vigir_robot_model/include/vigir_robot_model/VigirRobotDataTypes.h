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

#ifndef __VIGIR_ROBOT_DATA_TYPES_H__
#define __VIGIR_ROBOT_DATA_TYPES_H__

// Define some basic types to replace ROS specific messages
// These are defined using the Eigen library and are more suitable for
// direct calculation

// Define Eigen setup used by rbdl
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>

namespace vigir_control{

typedef Eigen::VectorXd             VectorNd;
typedef Eigen::MatrixXd             MatrixNd;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Vector3d             Vector3d;
typedef Eigen::Vector2d             Vector2d;
typedef Eigen::Quaternion<double>   Quatd;

#define quat_equals( q1, q2) ((q1.x() == q2.x()) && (q1.y() == q2.y()) && (q1.z() == q2.z()) && (q1.w() == q2.w()))


typedef struct Wrench
{
    Wrench() : force(0.0, 0.0, 0.0), torque(0.0, 0.0, 0.0) {}
    Vector3d    force;
    Vector3d    torque;

    bool operator== (const Wrench& rhs)
    {
        return ((force == rhs.force) && (torque == rhs.torque));
    }

    bool operator!= (const Wrench& rhs)
    {
        return ((force != rhs.force) || (torque != rhs.torque));

    }

} Wrench;

typedef struct IMU
{
    IMU() : orientation(1.0,0.0,0.0,0.0),angular_velocity(0.0,0.0,0.0),linear_acceleration(0.0,0.0,0.0){}

    Quatd    orientation;
    Vector3d angular_velocity;
    Vector3d linear_acceleration;

    bool operator== (const IMU& rhs)
    {
        return (quat_equals(orientation, rhs.orientation) && (angular_velocity == rhs.angular_velocity)  && (linear_acceleration == rhs.linear_acceleration));
    }

    bool operator!= (const IMU& rhs)
    {
        return (!quat_equals(orientation,rhs.orientation) || (angular_velocity != rhs.angular_velocity)  || (linear_acceleration != rhs.linear_acceleration));

    }
} IMU;

typedef struct Pose
{
    Pose() : position(0.0,0.0,0.0),orientation(1.0,0.0,0.0,0.0){}

    Vector3d position;
    Quatd    orientation;

    bool operator== (const Pose& rhs)
    {
        return ((position == rhs.position) && quat_equals(orientation,rhs.orientation));
    }

    bool operator!= (const Pose& rhs)
    {
        return ((position != rhs.position) || !quat_equals(orientation,rhs.orientation));

    }

    Pose operator *= (const Pose& rhs)
    {
        position += orientation*rhs.position;
        orientation *= rhs.orientation;
        return *this;
    }

    // Returns the inverse transformation as a pose
    void invert(Pose& lhs)
    {
        lhs.orientation = orientation.inverse();       // R^t
        lhs.position    = lhs.orientation*(-position); // R^t * -p
    }

} Pose;

typedef struct Twist
{
    Twist() : linear(0.0,0.0,0.0),angular(0.0,0.0,0.0){}

    Vector3d linear;
    Vector3d angular;

    bool operator== (const Twist& rhs)
    {
        return ((linear == rhs.linear) && (angular == rhs.angular));
    }

    bool operator!= (const Twist& rhs)
    {
        return ((linear != rhs.linear) || (angular != rhs.angular));

    }

} Twist;

typedef struct PoseZYX
{
    PoseZYX() : position(0.0,0.0,0.0),orientation(0.0,0.0,0.0){}

    Vector3d position;
    Vector3d orientation;

    bool operator== (const PoseZYX& rhs)
    {
        return ((position == rhs.position) && (orientation == rhs.orientation));
    }

    bool operator!= (const PoseZYX& rhs)
    {
        return ((position != rhs.position) || (orientation != rhs.orientation));

    }

} PoseZYX;

typedef struct Transform
{
    Transform() :translation(0.0,0.0,0.0){ rotation.setIdentity();}

    Vector3d translation;
    Matrix3d rotation;

    Vector3d operator *(const Vector3d& rhs) const
    {
        return rotation*rhs + translation;
    }

    bool operator== (const Transform& rhs)
    {
        return ((translation == rhs.translation) && (rotation == rhs.rotation));
    }

    bool operator!= (const Transform& rhs)
    {
        return ((translation != rhs.translation) || (rotation != rhs.rotation));

    }

    Transform operator *= (const Transform& rhs)
    {
        translation += rotation*rhs.translation;
        rotation *= rhs.rotation;
        return *this;
    }

    // Returns the inverse transformation assuming a SO(3) rotation matrix
    void invert(Transform& lhs)
    {
        lhs.rotation    = rotation.transpose();        // R^t
        lhs.translation = lhs.rotation*(-translation); // R^t * -p
    }

} Transform;

typedef std::vector<Vector3d>  Polygon3D;
typedef std::vector<Vector2d>  Polygon2D;


// std::vectors containing any objectst that have Eigen matrices or vectors
// as members need to have a special allocater. This can be achieved with
// the following macro.

//#ifdef EIGEN_CORE_H
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix6d);
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Vector3d);
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Wrench);
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IMU);
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Pose);
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Transform);
//#endif

}
#endif

