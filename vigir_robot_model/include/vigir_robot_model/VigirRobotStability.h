//=================================================================================================
// Copyright (c) 2013-2015, David Conner, TORC Robotics
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of TORC Robotics nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

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

#ifndef VIGIR_ROBOT_STABILITY_H__
#define VIGIR_ROBOT_STABILITY_H__

#include <vigir_robot_model/VigirRobotDataTypes.h>

namespace vigir_control
{
typedef enum {
    STANCE_UNKNOWN   = 0,
    RIGHT_LEG_STANCE = 0x01,
    RIGHT_LEG_SWING  = 0x02,
    LEFT_LEG_STANCE  = 0x10,
    LEFT_LEG_SWING   = 0x20,
    DOUBLE_SUPPORT   = 0x11,
    FLIGHT           = 0x22
} VigirStanceState;

typedef struct VigirStabilityData
{

    uint64_t                 timestamp;
    uint8_t                  stability_factor;  // 0 - 255 (most stable)
    VigirStanceState         stance_state;      //
    int8_t                   current_support_;  // -1 left, 1 right, 2=double
    bool                     stable;
    bool                     currently_falling;
    int8_t                   fall_state;        // -1 fallen, 0=stable upright, 1=currently falling

    Transform right_foot;
    Transform left_foot;
    Transform right_hand;
    Transform left_hand;
    double    mass;
    Vector3d  center_of_mass;
    Vector3d  center_of_pressure;
    Vector3d  CoP_left;
    Vector3d  CoP_right;

    Polygon2D support;

    // return falling if we are actively falling
    bool falling(){ return (currently_falling = (fall_state > 0));}

    // Temporary debugging data
    std::vector<Transform> joint_transforms;

    VigirStabilityData() : stability_factor(0), stable(true), mass(0.0), fall_state(0), currently_falling(false) {}

} VigirStabilityData;



  /**
    * This class defines the base class to perform various stability calculations necessary for walking and standing stablely.
    */
//  class FlorStability {

//  public:

//    FlorStability( );
//    virtual ~FlorStability();


//    virtual inline bool setControlStateCommand(const uint8_t& command_state)
//    {
//        commanded_state_ = command_state;
//        return false;
//    }

//    // Do the stability calculations given data pre-processed
//    void calculateStability();

//    /// Center of Mass in pelvis frame
//    flor_control::Vector3d    CoM() ;
//    double      mass();
//    void        getStability(uint64_t& timestamp, flor_control::Pose& pelvis_pose, double& mass,
//                             flor_control::Vector3d& CoM,       flor_control::Vector3d& CoM_gravity,
//                             flor_control::Vector3d& gravity,   uint8_t& stance, int8_t& support_state,
//                             flor_control::Vector3d& CoP_posn_right, flor_control::Vector3d& CoP_force_right,
//                             flor_control::Vector3d& CoP_posn_left , flor_control::Vector3d& CoP_force_left,
//                             flor_control::Polygon& support_polygon, flor_control::Vector3d& CoP_weighted,
//                             flor_control::Vector3d& ZMP_weighted,
//                             double& pelvis_height, uint8_t& stability);

//    void updateStateData( FlorDynamics& dynamics_);

//    void getFeetPosesWorld(flor_control::Pose& l_foot_pose, flor_control::Pose& r_foot_pose);

//    /**
//     * Notify derived classes that new stability data is ready.
//     * Can use this to publish data from derived class that knows about the ROS setup
//     */
//     virtual void updateStability(){};

//  protected:
//    protected: uint64_t                            timestamp_;

//    //protected: Quatd                             imu_to_pelvis_;          --- Assume identity for now
//    protected: flor_control::Quatd                 orientation_pelvis_;

//    protected: flor_control::Pose                  pelvis_pose_;           // last known pose in world frame
//    protected: flor_control::IMU                   imu_data_;
//    protected: flor_control::Vector3d              CoP_posn_right_;
//    protected: flor_control::Vector3d              CoP_posn_left_;
//    protected: flor_control::Vector3d              CoP_force_right_;
//    protected: flor_control::Vector3d              CoP_force_left_;
//    protected: flor_control::Vector3d              gravity_aligned_;

//    protected: double                              mass_;
//    protected: flor_control::Vector3d              CoM_pelvis_;
//    protected: flor_control::Vector3d              CoM_gravity_;            // Aligned with gravity vector
////    protected: flor_control::Wrench                external_wrench_;        // Wrench at CoM due to hand forces
//    protected: flor_control::Transform             r_foot_transform_;
//    protected: flor_control::Transform             l_foot_transform_;
//    protected: flor_control::Transform             r_hand_transform_;
//    protected: flor_control::Transform             l_hand_transform_;
//    protected: flor_control::Wrench                r_foot_wrench_;
//    protected: flor_control::Wrench                l_foot_wrench_;
//    protected: flor_control::Wrench                r_hand_wrench_;
//    protected: flor_control::Wrench                l_hand_wrench_;
//    protected: double                              pelvis_height_;


//    protected: uint64_t                            lastControllerUpdateTime_;
//    protected: uint8_t                             stance_state_;
//    protected: uint8_t                             active_state_;
//    protected: uint8_t                             commanded_state_;
//    protected: int8_t                              current_support_;        // -1 left, 1 right, 2=double
//    protected: uint8_t                             stability_;              // 0 - outside support, 255 - best case of CoP inside square support region  = 255 * (2*d_min/sqrt(A))

//    class SupportPolygonPrivate;
//    protected: boost::shared_ptr<SupportPolygonPrivate> current_support_polygon_;

//    protected: flor_control::Polygon               support_polygon_;            // include z value for visualization in rugged terrain, but this is effectively 2D projection onto ground
//    protected: flor_control::Vector3d              CoP_weighted_;               // relative to pelvis (gravitational frame)
//    protected: flor_control::Vector3d              ZMP_weighted_;               // relative to pelvis (gravitational frame)

//    protected: double                              foot_contact_threshold_max_;
//    protected: double                              foot_contact_threshold_min_;


//    Timing                                         calc_stability_timing_;

//  };

}

#endif

