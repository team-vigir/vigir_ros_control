//=================================================================================================
// Copyright (c) 2013, David Conner, TORC Robotics
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

#include <ros/ros.h>

#include <flor_dynamics/FlorStability.h>
#include <vigir_utilities/timing.h>

#include <boost/thread/locks.hpp>

#include <pcl/surface/convex_hull.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

namespace flor_control{

    // Define this private class to hide PCL-based implementation
    class FlorStability::SupportPolygonPrivate
    {
    public:
        SupportPolygonPrivate()
        {
            ROS_INFO(" Construct support polygon handler");
            {

                pcl::ConvexHull<pcl::PointXYZ> chull;

                ROS_INFO("Set up the right/left feet support polygons ...");

                // @TODO - remove vertices to reduce complexity
                z_sole_ = 0.0;
                boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > left_foot_pcl;
                left_foot_pcl.reset(new pcl::PointCloud<pcl::PointXYZ>() );
                left_foot_pcl->push_back(pcl::PointXYZ( 0.080193, -0.061646, 0.0)); z_sole_ += ((-0.081119));
                left_foot_pcl->push_back(pcl::PointXYZ(-0.081627, -0.037043, 0.0)); z_sole_ += ((-0.081119));
                left_foot_pcl->push_back(pcl::PointXYZ(-0.054600, -0.059442, 0.0)); z_sole_ += ((-0.081119));
                left_foot_pcl->push_back(pcl::PointXYZ(-0.048767, -0.061646, 0.0)); z_sole_ += ((-0.081119));
                left_foot_pcl->push_back(pcl::PointXYZ(-0.079127,  0.043641, 0.0)); z_sole_ += ((-0.081119));
                left_foot_pcl->push_back(pcl::PointXYZ(-0.074960,  0.048426, 0.0)); z_sole_ += ((-0.081119));
                left_foot_pcl->push_back(pcl::PointXYZ(-0.081627,  0.038103, 0.0)); z_sole_ += ((-0.081119));
                left_foot_pcl->push_back(pcl::PointXYZ(-0.074960, -0.047367, 0.0)); z_sole_ += ((-0.081119));
                left_foot_pcl->push_back(pcl::PointXYZ(-0.066266, -0.055032, 0.0)); z_sole_ += ((-0.081119));
                left_foot_pcl->push_back(pcl::PointXYZ(-0.079127, -0.042581, 0.0)); z_sole_ += ((-0.08111));
                left_foot_pcl->push_back(pcl::PointXYZ(-0.082460, -0.030753, 0.0)); z_sole_ += ((-0.081119));
                left_foot_pcl->push_back(pcl::PointXYZ( 0.179390,  0.026279, 0.0)); z_sole_ += ((-0.080325));
                left_foot_pcl->push_back(pcl::PointXYZ( 0.179390, -0.025219, 0.0)); z_sole_ += ((-0.080325));
                left_foot_pcl->push_back(pcl::PointXYZ(-0.082460,  0.031812, 0.0)); z_sole_ += ((-0.081119));
                left_foot_pcl->push_back(pcl::PointXYZ( 0.169727,  0.038853, 0.0)); z_sole_ += ((-0.080403));
                left_foot_pcl->push_back(pcl::PointXYZ( 0.175095, -0.035252, 0.0)); z_sole_ += ((-0.08036));
                left_foot_pcl->push_back(pcl::PointXYZ( 0.175095,  0.036311, 0.0)); z_sole_ += ((-0.08036));
                left_foot_pcl->push_back(pcl::PointXYZ( 0.169727, -0.037793, 0.0)); z_sole_ += ((-0.080403));
                left_foot_pcl->push_back(pcl::PointXYZ(-0.060433,  0.058298, 0.0)); z_sole_ += ((-0.08112));
                left_foot_pcl->push_back(pcl::PointXYZ(-0.066266,  0.056093, 0.0)); z_sole_ += ((-0.08112));
                left_foot_pcl->push_back(pcl::PointXYZ( 0.080193,  0.062707, 0.0)); z_sole_ += ((-0.08112));
                left_foot_pcl->push_back(pcl::PointXYZ(-0.048767,  0.062707, 0.0)); z_sole_ += ((-0.08112));
                left_foot_pcl->push_back(pcl::PointXYZ( 0.178316,  0.032120, 0.0)); z_sole_ += ((-0.080334));
                left_foot_pcl->push_back(pcl::PointXYZ( 0.178316, -0.031060, 0.0)); z_sole_ += ((-0.080334));
                size_t n_pts = left_foot_pcl->size();
                z_sole_ /= double(n_pts);

#if ROS_VERSION_MINIMUM(1,8,0) // test for Fuerte (newer PCL)
                chull.setDimension(2);
#endif
                chull.setComputeAreaVolume(true);
                chull.setInputCloud(left_foot_pcl);
                std::vector<pcl::Vertices>      polygons;
                pcl::PointCloud<pcl::PointXYZ>  chull_points;
                chull.reconstruct(chull_points, polygons);
                left_foot_polygon_.resize(polygons[0].vertices.size());
                sqrt_foot_area_ = sqrt(chull.getTotalArea());

                for (unsigned i = 0; i < polygons[0].vertices.size(); ++i){
                      int idx = polygons[0].vertices[i];
                      left_foot_polygon_[i].x() = chull_points.points[idx].x;
                      left_foot_polygon_[i].y() = chull_points.points[idx].y;
                      left_foot_polygon_[i].z() = z_sole_;
                 }
                ROS_INFO(" Left foot polygon with %ld vertices (%ld originally) z_sole=%f", left_foot_polygon_.size(), n_pts, this->z_sole_);

                ROS_ERROR("     Current model uses 2 left feet - verify model ! ");
                right_foot_polygon_ = left_foot_polygon_;
                ROS_INFO(" Right foot polygon with %d vertices", uint32_t(right_foot_polygon_.size()));
            }
        }

        ~SupportPolygonPrivate()
        {
            ROS_INFO(" pcl::PointCloud<pcl::PointXYZ>Destroy support polygon handler");

        }

        // Returns stability metric 0=unstable, 255= best case
        uint8_t calculateSupport(const int8_t& current_support, const Transform& l_foot_transform, const Transform& r_foot_transform,
                              const Vector3d& CoP_posn_left, const Vector3d& CoP_posn_right, const Quatd& quat, const double& right_weight)
        {
            Transform rot;
            uint8_t stability = 0;
            switch(current_support)
            {
            case 1:
                current_support_polygon_.resize(right_foot_polygon_.size());
                rot.translation = quat*r_foot_transform.translation;
                rot.rotation    = quat*r_foot_transform.rotation;

                for (uint32_t ndx=0; ndx < right_foot_polygon_.size(); ++ndx)
                {
                    current_support_polygon_[ndx] = rot*right_foot_polygon_[ndx];  // vertex in pelvis gravitational frame
                }
                CoP_weighted_  = rot*CoP_posn_right;
                pelvis_height_ = -rot.translation(2);
                //ROS_INFO("Right support polygon = %ld pts z_pelvis = %f", current_support_polygon_.size(), pelvis_height_);

                return pointInConvexHull(CoP_weighted_);
                break;
            case -1:
                rot.translation = quat*l_foot_transform.translation;
                rot.rotation    = quat*l_foot_transform.rotation;
                current_support_polygon_.resize(left_foot_polygon_.size());
                for (uint32_t ndx=0; ndx < left_foot_polygon_.size(); ++ndx)
                {
                    current_support_polygon_[ndx] = rot*left_foot_polygon_[ndx];  // vertex in pelvis gravitational frame
                }
                CoP_weighted_  = rot*CoP_posn_left;
                pelvis_height_ = -rot.translation(2);
                //ROS_INFO("Left support polygon = %ld pts z_pelvis = %f", current_support_polygon_.size(), pelvis_height_);

                return pointInConvexHull(CoP_weighted_);
                break;
            case 0:
                {
                // Double support, need to find the convex hull of points
                    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > support_pcl;
                    support_pcl.reset(new pcl::PointCloud<pcl::PointXYZ>() );
                    support_pcl->resize(left_foot_polygon_.size() + right_foot_polygon_.size());
                    flor_control::Vector3d vec;
                    rot.translation = quat*l_foot_transform.translation;
                    rot.rotation    = quat*l_foot_transform.rotation;
                    pelvis_height_  = -rot.translation(2)*(1.0-right_weight);

                    for (uint32_t ndx=0; ndx < left_foot_polygon_.size(); ++ndx)
                    {
                        vec = rot*left_foot_polygon_[ndx];  // vertex in pelvis gravitational frame
                        support_pcl->at(ndx).x = vec.x();
                        support_pcl->at(ndx).y = vec.y();
                        support_pcl->at(ndx).z = vec.z();
                    }

                    rot.translation = quat*r_foot_transform.translation;
                    rot.rotation    = quat*r_foot_transform.rotation;
                    pelvis_height_ += -rot.translation(2)*right_weight;

                    for (uint32_t ndx=0; ndx < right_foot_polygon_.size(); ++ndx)
                    {
                        vec = rot*right_foot_polygon_[ndx];  // vertex in pelvis gravitational frame
                        uint32_t indx = left_foot_polygon_.size() + ndx;
                        support_pcl->at(indx).x = vec.x();
                        support_pcl->at(indx).y = vec.y();
                        support_pcl->at(indx).z = vec.z();
                    }

                    // extract convex
                    //ROS_INFO("Extract convex hull for double support");
                    pcl::ConvexHull<pcl::PointXYZ> chull;
#if ROS_VERSION_MINIMUM(1,8,0) // test for Fuerte (newer PCL)
                    chull.setDimension(2);  // want to ignore z in calculating the points
#endif
                    chull.setInputCloud(support_pcl);
                    std::vector<pcl::Vertices>      polygons;
                    pcl::PointCloud<pcl::PointXYZ>  chull_points;
                    chull.reconstruct(chull_points, polygons);

                    current_support_polygon_.resize(polygons[0].vertices.size());
                    for (unsigned i = 0; i < polygons[0].vertices.size(); ++i){
                          int idx = polygons[0].vertices[i];
                          current_support_polygon_[i].x() = chull_points.points[idx].x;
                          current_support_polygon_[i].y() = chull_points.points[idx].y;
                          current_support_polygon_[i].z() = chull_points.points[idx].z;
                          //ROS_INFO("  SPv % 3d : %f, %f, %f", i, current_support_polygon_[i].x(), current_support_polygon_[i].y(), current_support_polygon_[i].z());
                     }
                    //ROS_INFO("  Convex hull # points in =%ld  output=%ld  %ld", support_pcl->size(), chull_points.size(), polygons[0].vertices.size());

                    Vector3d rt = r_foot_transform*CoP_posn_right; // transform from foot frame to pelvis
                    Vector3d lt = l_foot_transform*CoP_posn_left;
                    CoP_weighted_ = quat*( right_weight*rt + (1.0 - right_weight)*lt ); // weighted by the support effort, and rotated to gravitational frame
//                    ROS_INFO("Double support |fp|=%ld |pcl|=%ld |chull|=%ld |poly|=%ld right_weight=%f CoP = (%f, %f, %f) = (%f, %f, %f) + (%f, %f, %f)  T=(%f, %f)",
//                             left_foot_polygon_.size(),
//                             support_pcl->size(),
//                             chull_points.points.size(),
//                             current_support_polygon_.size(),
//                             right_weight,
//                             CoP_weighted_.x(), CoP_weighted_.y(), CoP_weighted_.z(),
//                             lt.x(), lt.y(), lt.z(), rt.x(), rt.y(), rt.z() ,
//                             l_foot_transform.translation.z(), r_foot_transform.translation.z());
                    stability = pointInConvexHull(CoP_weighted_);
                    if (0 == stability)
                    {
//                        ROS_INFO("\n\n\n\n ----------------------------------- Unstable save support data -----------------------------------");
//                        std::stringstream ss;
//                        ss << "/home/vigir/test_" << ros::Time::now().toNSec() << "_";
//                        pcl::io::savePCDFileASCII ((ss.str() + "input_pcd.pcd"),  *support_pcl);
//                        pcl::io::savePCDFileASCII ((ss.str() + "output_pcd.pcd"), chull_points);
//                        std::string poly_str = "support_poly=np.array([";

                        for (unsigned i = 0; i < support_pcl->size(); ++i){
//                          std::stringstream ss;
//                          ss << "[" << support_pcl->at(i).x << ", " << support_pcl->at(i).y << ", " << support_pcl->at(i).z <<"], ";
//                          poly_str += ss.str();
                          support_pcl->at(i).z = 0.0; // reset to 0.0
                        }
//                        poly_str += "])\n";
//                        ROS_INFO(" \n  %s\n---------------------------",poly_str.c_str());
//                        poly_str = "hull_poly=np.array([";

//                        for (unsigned i = 0; i < current_support_polygon_.size(); ++i){
//                          std::stringstream ss;
//                          ss << "[" << current_support_polygon_[i].x() << ", " << current_support_polygon_[i].y() << ", " << current_support_polygon_[i].z() <<"], ";
//                          poly_str += ss.str();

//                        }
//                        poly_str += "])\n";
//                        ss.seekp(0); // reset
//                        ss << "CoP=np.array([[" << CoP_weighted_.x() << ", " << CoP_weighted_.y() << ", " << CoP_weighted_.z() << "]])\n";
//                        ROS_INFO(" \n  %s\n %s\n ---------------------------",poly_str.c_str(),ss.str().c_str());
//                        ROS_INFO("\n\n ----------------------------------- Unstable saved support data complete! -----------------------------------\n\n\n");

                        // Retry - ---
                        chull.setInputCloud(support_pcl);
                        chull.reconstruct(chull_points, polygons);

                        current_support_polygon_.resize(polygons[0].vertices.size());
                        for (unsigned i = 0; i < polygons[0].vertices.size(); ++i){
                              int idx = polygons[0].vertices[i];
                              current_support_polygon_[i].x() = chull_points.points[idx].x;
                              current_support_polygon_[i].y() = chull_points.points[idx].y;
                              current_support_polygon_[i].z() = -pelvis_height_ + right_foot_polygon_[0].z();
                              //ROS_INFO("  SPv % 3d : %f, %f, %f", i, current_support_polygon_[i].x(), current_support_polygon_[i].y(), current_support_polygon_[i].z());
                         }
                        stability = pointInConvexHull(CoP_weighted_);
                        ROS_DEBUG("  Initial stability test failed - after z=0  stability = %d\n\n\n", stability);
                        //while(1){ros::spinOnce();}
                    }

                }
                break;

            case -2:
            default:
                ROS_DEBUG("Invalid support %d polygon = %ld pts z_pelvis = %f", current_support, current_support_polygon_.size(), pelvis_height_);

                current_support_polygon_.clear();
                CoP_weighted_.setZero();
                pelvis_height_ = 0.0;
                return 0;
            }

            // OK - now we have support polygon in pelvis frame, rotate to align with gravitational field

            return stability;
        }

        // This checks for inclusion and uses distance to boundary to calculate stability metric
        uint8_t pointInConvexHull(const flor_control::Vector3d& point) const{

          if (current_support_polygon_.size() < 3 ) return 0; // unstable

          int positive_direction = 0;
          double min_dist = sqrt_foot_area_;
          Vector3d dV,dP;
          dV(2) = 0.0; // z-component for dot product

          for (unsigned i = 0; i < current_support_polygon_.size(); ++i){
            int i2 = (i+1)% (current_support_polygon_.size());
            dV(0) = current_support_polygon_[i2].x() - current_support_polygon_[i].x();
            dV(1) = current_support_polygon_[i2].y() - current_support_polygon_[i].y();
            if (dV(0) == 0.0 && dV(1) == 0.0){
              //ROS_ERROR("Skipping polygon connection [%d-%d] (identical points)  (%f, %f) vs. (%f, %f)", i, i2,
              //          current_support_polygon_[i2].x(),current_support_polygon_[i2].y(),current_support_polygon_[i].x(),current_support_polygon_[i].y());
              continue;
            }
            dV.normalize();
            dP = (point - current_support_polygon_[i]); // distance to boundary (point.y() - current_support_polygon_[i].y())*dx - (point.x() - current_support_polygon_[i].x())*dy;
            double line_test = -dV.y()*dP.x() + dV.x()*dP.y(); // rotate boundary to normal position
            if (i == 0)
              positive_direction = (line_test > 0.0);
            //ROS_DEBUG("Line test [%d-%d] from (%f,%f) to (%f,%f): dV=(%f,%f) dP=(%f,%f) d=%f", i, i2, current_support_polygon_[i].x(), current_support_polygon_[i].y(),
            //          current_support_polygon_[i2].x(), current_support_polygon_[i2].y(),
            //         dV.x(), dV.y(), dP.x(), dP.y(), line_test);
            if ((line_test > 0.0) != positive_direction)
            {
                //ROS_INFO("CoP outside polygon");
                //ROS_INFO("Line test [%d-%d] from (%f,%f) to (%f,%f): dV=(%f,%f) dP=(%f,%f) d=%f", i, i2, current_support_polygon_[i].x(), current_support_polygon_[i].y(),
                //          current_support_polygon_[i2].x(), current_support_polygon_[i2].y(),
                //          dV.x(), dV.y(), dP.x(), dP.y(), line_test);

                return 0; // swapped sides must be outside
            }

            if (fabs(line_test) < min_dist) min_dist= fabs(line_test);

          }

          double test = 255.0*2.0*(min_dist/sqrt_foot_area_);
          if (test > 255.0) return 255;
          return uint8_t(test);
        }

        void extract2DSupport(flor_control::Polygon& support)
        {
            support = current_support_polygon_;
        }

        double                 z_sole_;
        double                 pelvis_height_;
        Vector3d               CoP_weighted_;
        flor_control::Polygon  current_support_polygon_;    // normalized to pelvis aligned in gravity frame

    protected:

        flor_control::Polygon  right_foot_polygon_;
        flor_control::Polygon  left_foot_polygon_;
        double                 sqrt_foot_area_;

    };


    FlorStability::FlorStability( )
        : stance_state_(STANCE_UNKNOWN),
          active_state_(STATE_UNKNOWN),
          foot_contact_threshold_max_(12.0*9.81),
          foot_contact_threshold_min_( 2.5*9.81),
          calc_stability_timing_("FlorStability: calc stability timing")
    {
        ROS_INFO("Set up the robot data for the stability calculations ...");

        // Define the support polygons at feet
        current_support_polygon_.reset(new FlorStability::SupportPolygonPrivate());

        // Initialize the internal data
        // initialize update time
        ROS_WARN("Initialize the internal data ...");
        this->lastControllerUpdateTime_ = 0;

    }

    FlorStability::~FlorStability()
    {
        ROS_WARN("Shutting down the FlorStability ...");
    }


    void FlorStability::getStability(uint64_t& timestamp, flor_control::Pose& pelvis_pose, double& mass,
                             flor_control::Vector3d& CoM,       flor_control::Vector3d& CoM_gravity,
                             flor_control::Vector3d& gravity,   uint8_t& stance, int8_t& support_state,
                             flor_control::Vector3d& CoP_posn_right, flor_control::Vector3d& CoP_force_right,
                             flor_control::Vector3d& CoP_posn_left , flor_control::Vector3d& CoP_force_left,
                             flor_control::Polygon& support_polygon, flor_control::Vector3d& CoP_weighted,
                             flor_control::Vector3d& ZMP_weighted,
                             double& pelvis_height, uint8_t& stability)
    {
        timestamp       = this->timestamp_;
        pelvis_pose     = this->pelvis_pose_;
        pelvis_pose.orientation = this->orientation_pelvis_; //
        mass            = this->mass_;
        CoM             = this->CoM_pelvis_;
        CoM_gravity     = this->CoM_gravity_;
        gravity         = this->gravity_aligned_;
        stance          = this->stance_state_;
        support_state   = this->current_support_;

        // Return center of pressure data (coincides with ZMP if we're in good contact)
        CoP_posn_right  = this->CoP_posn_right_;
        CoP_force_right = this->CoP_force_right_;
        CoP_posn_left   = this->CoP_posn_left_;
        CoP_force_left  = this->CoP_force_left_;
        support_polygon = this->support_polygon_;
        CoP_weighted    = this->CoP_weighted_;
        ZMP_weighted    = this->ZMP_weighted_;
        pelvis_height   = this->pelvis_height_;
        stability       = this->stability_;
        return;
    }

    // Find the Center of Mass (CoM) and transforms for feet and hands all in pelvis frame
    // This function should be called from thread where both Stability and Dynamic data is protected
    void FlorStability::updateStateData(FlorDynamics& dynamics)
    {
        double mass;
        dynamics.getKinematics( this->timestamp_,
                                this->CoM_pelvis_,
                                this->mass_,
                                this->r_foot_transform_,
                                this->l_foot_transform_ ,
                                this->r_hand_transform_,
                                this->l_hand_transform_,
                                this->gravity_aligned_,
                                this->orientation_pelvis_);

        dynamics.getState(this->r_foot_wrench_, this->l_foot_wrench_,
                          this->r_hand_wrench_, this->l_hand_wrench_,
                          this->pelvis_pose_,   this->imu_data_);

        // Temporary debug test
        if (std::isnan(this->r_foot_wrench_.force.z()) ||
            std::isnan(this->r_foot_wrench_.torque.x()) ||
            std::isnan(this->r_foot_wrench_.torque.y()) ||
            std::isnan(this->l_foot_wrench_.torque.x()) ||
            std::isnan(this->l_foot_wrench_.torque.y()) ||
            std::isnan(this->l_foot_wrench_.force.z()))
        {
            ROS_ERROR("FlorStability::updateKinematicData - nan!  (%f, %f, %f)  (%f, %f, %f) ",
                      this->l_foot_wrench_.torque.x(), this->l_foot_wrench_.torque.y(), this->l_foot_wrench_.force.z(),
                      this->r_foot_wrench_.torque.x(), this->r_foot_wrench_.torque.y(), this->r_foot_wrench_.force.z());
        }

    }

    // Performs stability calculations assuming that sensor data is processed
    void  FlorStability::calculateStability()
    {
        DO_TIMING( calc_stability_timing_ );
        //calc_stability_timing_.start();

            // Get Orientation of pelvis from IMU
            //Assume fixed alignment with no rotation // Quatd pelvis_orientation = imu_to_pelvis_*this->filtered_orientation_;// rotate IMU frame into pelvis frame

            // Rotate body frame into world frame
            Quatd& quat = this->orientation_pelvis_;//.inverse();//(1.0,0.0,0.0,0.0); // identity

            Vector3d CoM_gravity     = quat*Vector3d(this->CoM_pelvis_.x(),this->CoM_pelvis_.y(),this->CoM_pelvis_.z());

            // Calculate the CoP position and equivalent forces in foot frame assuming contact
            // Stability is only relvant for actual contact

            // Force torque values on foot at joint ankle are Fz, Mx, and My in the foot frame
            // Sum of reaction forces on foot Rz + Fz = 0 ==> Rz = -Fz
            // Sum of moments on foot at ankle joint CoP(cx,cy,z_sole) x R(0,0,Rz) + M(Mx,My,0)  = 0
            // CoP x R = [(cy*Rz - cz*0), (cz*0 - cx*Rz), (cx*0 - cy*0)]
            // [ cyRz, -cx*Rz,0] = [-Mx,-My,0]
            // CoP=([cx,cy,z_sole] = [My/Rz,-Mx/Rz,z_sole]
            // This agrees with COP = nxM/R with n=OUTWARD pointing normal of sole

            double Rz =  this->r_foot_wrench_.force.z();
            double Mx = -this->r_foot_wrench_.torque.x();
            double My = -this->r_foot_wrench_.torque.y();
            Vector3d  CoP_posn_right(  My/Rz, -Mx/Rz,current_support_polygon_->z_sole_);
            Vector3d  CoP_force_right( Mx,     My,      Rz);
            //ROS_INFO("  Right: forces (%f, %f, %f)  posn(%f, %f, %f)",
            //         CoP_force_right(0),CoP_force_right(1),CoP_force_right(2),
            //         CoP_posn_right(0), CoP_posn_right(1), CoP_posn_right(2));

            Rz =  this->l_foot_wrench_.force.z();
            Mx = -this->l_foot_wrench_.torque.x();
            My = -this->l_foot_wrench_.torque.y();
            Vector3d  CoP_posn_left( My/Rz, -Mx/Rz,current_support_polygon_->z_sole_);
            Vector3d  CoP_force_left( Mx,    My,   Rz);
            //ROS_INFO("  Left: forces (%f, %f, %f)  posn(%f, %f, %f)",
            //         CoP_force_left(0), CoP_force_left(1),CoP_force_left(2),
            //         CoP_posn_left(0),  CoP_posn_left(1), CoP_posn_left(2)  );
            if (std::isnan(CoP_force_left.z()) || std::isnan(CoP_force_right.z()))
            {
                ROS_ERROR("FlorStability::calculateStability - (%f, %f, %f)  (%f, %f, %f) ",
                          CoP_force_left.x(), CoP_force_left.y(), CoP_force_left.z(),
                          CoP_force_right.x(), CoP_force_right.y(), CoP_force_right.z());
            }
            if (std::isnan(CoP_posn_left.z()) || std::isnan(CoP_posn_right.z()))
            {
                ROS_ERROR("FlorStability::calculateStability - (%f, %f, %f)  (%f, %f, %f) ",
                          CoP_posn_left.x(),  CoP_posn_left.y(),  CoP_posn_left.z(),
                          CoP_posn_right.x(), CoP_posn_right.y(), CoP_posn_right.z());
            }

            uint8_t stance_state = this->stance_state_;
            if (CoP_force_right.z() > foot_contact_threshold_max_)
            { // Definite contact
                //ROS_INFO("Stance - right contact");
                stance_state &= ~RIGHT_LEG_SWING; // clear the swing bits
                stance_state |=  RIGHT_LEG_STANCE;
            }
            else if (CoP_force_right.z() > foot_contact_threshold_min_)
            {// Uncertain contact
                // leave it be
                //ROS_INFO("Stance - uncertain right contact");
            }
            else
            {  // No contact
               //ROS_INFO("Stance - no right contact");
               stance_state &= ~FlorStability::RIGHT_LEG_STANCE;// clear the stance bits
               stance_state |=  FlorStability::RIGHT_LEG_SWING;
            }

            if (CoP_force_left.z() > foot_contact_threshold_max_)
            { // Definite contact
                stance_state &= ~FlorStability::LEFT_LEG_SWING; // clear the swing bits
                stance_state |=  FlorStability::LEFT_LEG_STANCE;
                //ROS_INFO("Stance - left contact");
            }
            else if (CoP_force_left.z() > foot_contact_threshold_min_)
            {// Uncertain contact
                // leave it be
                //ROS_INFO("Stance - uncertain left contact");
            }
            else
            {  // No contact
               stance_state &= ~FlorStability::LEFT_LEG_STANCE;// clear the stance bits
               stance_state |=  FlorStability::LEFT_LEG_SWING;
               //ROS_INFO("Stance - no left contact");
            }

            /// calculates support mode -1 for left foot, 0 for unknown, and +1 for right foot, and +2 for double support
            double right_weight = 0.0;
            uint8_t support = (stance_state_ & DOUBLE_SUPPORT); // mask swing state
            int8_t  current_support = -2;
            if (DOUBLE_SUPPORT == support)
            {
                current_support = 0;
                right_weight = CoP_force_right(2)/(CoP_force_left(2) + CoP_force_right(2));
                if (right_weight < 0.0) right_weight = 0.0;
                else if (right_weight > 1.0) right_weight = 1.0;
            }
            else if (support & RIGHT_LEG_STANCE)
            {
                current_support = 1;
                right_weight    = 1.0;
            }
            else if (support & LEFT_LEG_STANCE)
            {
                current_support = -1;
            }
            else
            {
                if (this->stance_state_ & DOUBLE_SUPPORT)
                {
                    // Maintain the prior support state if either was active
                    stance_state    = this->stance_state_;
                    current_support = this->current_support_;
                }

            }

//            ROS_INFO("Stance - state 0x%x  support=%d r_foot=(%f, %f, %f) l_foot=(%f, %f, %f)",
//                     stance_state, current_support,
//                     CoP_force_right(0),CoP_force_right(1),CoP_force_right(2),
//                     CoP_force_left(0),CoP_force_left(1),CoP_force_left(2));


            // Calculate the 2D support polygon relative to the current gravity vector

            uint8_t stability = current_support_polygon_->calculateSupport(current_support, this->l_foot_transform_, this->r_foot_transform_,
                                                                          CoP_posn_left,CoP_posn_right, quat, right_weight);
            if (//(stability < 50) ||
                    std::isnan(CoP_weighted_.x()) ||
                    std::isnan(CoP_weighted_.y()) ||
                    std::isnan(CoP_weighted_.z())
               )// || (current_support != this->current_support_)  || (stance_state != this->stance_state_))
            {
                ROS_INFO("-------------------------\n    stability = %d stance_state=0x%x  current_support=%d  \n     forces L(%f, %f, %f) R(%f, %f, %f) \n        CoP L(%f, %f, %f) R(%f, %f, %f) CoP(%f) = (%f, %f, %f) pelvis height=%f",
                     stability, stance_state, current_support,
                     CoP_force_left(0), CoP_force_left(1),CoP_force_left(2), CoP_force_right(0),CoP_force_right(1),CoP_force_right(2),
                     CoP_posn_left(0),  CoP_posn_left(1), CoP_posn_left(2),  CoP_posn_right(0), CoP_posn_right(1), CoP_posn_right(2),
                     right_weight, this->current_support_polygon_->CoP_weighted_.x(), this->current_support_polygon_->CoP_weighted_.y(), this->current_support_polygon_->CoP_weighted_.z(),
                     this->current_support_polygon_->pelvis_height_);
            }

            // Write the data that can be accessed from outside thread
            {
                //boost::lock_guard<boost::mutex> guard(this->calc_stability_mutex_);
                this->CoM_gravity_     = CoM_gravity;
                this->CoP_posn_right_  = CoP_posn_right  ;
                this->CoP_posn_left_   = CoP_posn_left   ;
                this->CoP_force_right_ = CoP_force_right ;
                this->CoP_force_left_  = CoP_force_left  ;
                this->stance_state_    = stance_state;
                this->current_support_ = current_support;
                current_support_polygon_->extract2DSupport(this->support_polygon_ );
                this->stability_       = stability;
                this->pelvis_height_   = this->current_support_polygon_->pelvis_height_;
                this->CoP_weighted_    = this->current_support_polygon_->CoP_weighted_;
                this->ZMP_weighted_    = this->CoP_weighted_; // @TODO - add ZMP calculation using gravitational data
            }
            // Do the state change calculations (if mode changed)
            //calc_stability_timing_.diff();
            //calc_stability_timing_.printStats();
    }

    // This function assumes that code is thread protected and prior structures are updated with current data
    void FlorStability::getFeetPosesWorld(flor_control::Pose& l_foot_pose, flor_control::Pose& r_foot_pose)
    {
        l_foot_pose.position    = this->pelvis_pose_.orientation * this->l_foot_transform_.translation + this->pelvis_pose_.position;
        l_foot_pose.orientation = this->pelvis_pose_.orientation * this->l_foot_transform_.rotation;
        r_foot_pose.position    = this->pelvis_pose_.orientation * this->r_foot_transform_.translation + this->pelvis_pose_.position;
        r_foot_pose.orientation = this->pelvis_pose_.orientation * this->r_foot_transform_.rotation;
    }


} // flor_control namespace
