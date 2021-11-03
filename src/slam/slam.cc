//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    slam.cc
\brief   SLAM Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "slam.h"

#include "vector_map/vector_map.h"


DEFINE_double(min_odom_loc_diff, .5, "Minimum Odom translation diff to create a new pose");
DEFINE_double(min_odom_angle_diff, 30, "Minimum Odom rotation diff to create a new pose");

DEFINE_double(sd_laser, 0.5, "Std dev of laser scan");
DEFINE_double(sd_odom_x, 5.0, "Std dev of odometry in x direction");
DEFINE_double(sd_odom_y, 5.0, "Std dev of odometry in x direction");
DEFINE_double(sd_odom_angle, 5.0, "Std dev of odometry in x direction");

DEFINE_double(raster_map_dist, 10.0, "Maximum distance of x & y axes in the rasterized map");
DEFINE_double(raster_pixel_dist, 0.2, "Size of each pixel in the raster map");
DEFINE_double(csm_transl_max, .5, "Max translation for CSM");
DEFINE_double(csm_angle_max, 15, "Max rotation for CSM");
DEFINE_double(csm_transl_step, 0.1, "Translation step size for CSM");
DEFINE_double(csm_angle_step, 0.1, "Rotation step size for CSM");


using namespace math_util;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;
using math_util::DegToRad;
using math_util::RadToDeg;
using math_util::AngleDiff;
using math_util::AngleMod;

namespace slam {

SLAM::SLAM() :
    num_pixels_((int)((2. * FLAGS_raster_map_dist)/FLAGS_raster_pixel_dist)),
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    current_odom_loc_(0, 0),
    current_odom_angle_(0),
    pose_initialized_(false) {
    }

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  if (!pose_initialized_){
    *loc = Eigen::Vector2f(0., 0.);
    *angle = 0;
  }
  auto prev_pose = prev_poses_.back();
  *loc = prev_pose.loc;
  *angle = prev_pose.angle;
}

std::vector<Eigen::Vector2f> SLAM::ScanToPointCloud(
                        const std::vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  
  std::vector<Eigen::Vector2f> point_cloud;

  const int num_rays = (int) ranges.size();
  const Vector2f kLaserLoc(0.2, 0); // Relative to car
  const float angle_increment = (angle_max - angle_min)/(num_rays-1);

  for (int i = 0; i < num_rays; i++) {
    float range = ranges[i];
    if (range >= range_min && range <= range_max) {
      float angle = angle_min + i * angle_increment;
      Vector2f laser_loc = range * Vector2f(cos(angle), sin(angle)) + kLaserLoc;
      point_cloud.push_back(laser_loc);
    }
  }

  return point_cloud;

}

void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.
  
  // Add a new pose when odom dist is >50cm or odom angle diff >30deg

  if (!pose_initialized_) {
    std::vector<Eigen::Vector2f> point_cloud = ScanToPointCloud(ranges, range_min, range_max, angle_min, angle_max);
    prev_poses_.push_back(PoseData{Eigen::Vector2f(0,0), 0, point_cloud});
    prev_point_cloud_ = point_cloud;
    pose_initialized_ = true;
  } else {

    float angle_diff = AngleDiff(current_odom_angle_, prev_odom_angle_);
    Eigen::Vector2f loc_diff = current_odom_loc_ - prev_odom_loc_;

    if (odom_initialized_ && (loc_diff.norm() > FLAGS_min_odom_loc_diff || RadToDeg(abs(angle_diff)) < FLAGS_min_odom_angle_diff)) {

      // Compute best pose
      std::vector<Eigen::Vector2f> point_cloud = ScanToPointCloud(ranges, range_min, range_max, angle_min, angle_max);
      
      Eigen::MatrixXf raster = RasterizePointCloud(prev_point_cloud_);

      SLAM::CsmData csm_data = CSM(point_cloud, raster);
      
      // Convert to SLAM global frame from previous
      PoseData pose;
      Eigen::Rotation2Df rotation_pose(previous_poses_.back().angle);
      pose.angle = AngleMod(csm_data.angle + previous_poses_.back().angle);
      pose.loc = rotation_pose * csm_data.loc + previous_poses_.back().loc;
      pose.point_cloud = point_cloud;

      Eigen::Rotation2Df rotation_new_pose(pose.angle);
      for (auto& point : pose.point_cloud) {
        point = rotation_new_pose * point + pose.loc;
      }
      

      // Add new pose
      prev_poses_.push_back(pose);

      prev_point_cloud_ = point_cloud;

      // Update Odom
      prev_odom_angle_ = current_odom_angle_;
      prev_odom_loc_ = current_odom_loc_;
    }
  }

}

Eigen::MatrixXf SLAM::RasterizePointCloud(const vector<Eigen::Vector2f> point_cloud) {
  
  Eigen::MatrixXf raster = Eigen::MatrixXf::Constant(num_pixels_, num_pixels_, std::numeric_limits<float>::lowest());

  // Compute max log likelihood of every point in the map (max from each point in point cloud)
  uint num_points = point_cloud.size();
  const float gaussian_pdf_const = -log(FLAGS_sd_laser) - 0.5 * log(2 * M_PI);
  for (uint pt_ind = 0; pt_ind < num_points; pt_ind++) {
    Eigen::Vector2f pt = point_cloud[pt_ind];
    
    for (uint x_ind = 0; x_ind < num_pixels_; x_ind++) {
      for (uint y_ind = 0; y_ind < num_pixels_; y_ind++) {
        
        Eigen::Vector2f loc = FLAGS_raster_pixel_dist * Eigen::Vector2f(x_ind, y_ind).array() - FLAGS_raster_map_dist;
        float dist = (loc - pt).norm();

        // Log of gaussian pdf
        float log_likelihood = gaussian_pdf_const - 0.5 * pow(dist/FLAGS_sd_laser, 2);
        
        raster(x_ind,y_ind) = std::max(raster(x_ind,y_ind), log_likelihood);
      }
    }

  }

  return raster;
}

SLAM::CsmData SLAM::CSM(const vector<Eigen::Vector2f> point_cloud, Eigen::MatrixXf raster) {
  
  // Iterate through angle, dx, and dy
  // // Center the search around our odomety position

  // These are constants for the log gaussian pdf
  const float odom_angle_pdf_const = -log(FLAGS_sd_odom_angle) - 0.5 * log(2 * M_PI);
  const float odom_x_pdf_const = -log(FLAGS_sd_odom_x) - 0.5 * log(2 * M_PI);
  const float odom_y_pdf_const = -log(FLAGS_sd_odom_y) - 0.5 * log(2 * M_PI);

  const Eigen::Rotation2Df rotation_odom(-prev_odom_angle_);
  Eigen::Vector2f rel_odom_loc = rotation_odom * (current_odom_loc_ - prev_odom_loc_);
  float rel_odom_angle = AngleDiff(current_odom_angle_, prev_odom_angle_);

  SLAM::CsmData results = SLAM::CsmData{rel_odom_loc,
                                        rel_odom_angle,
                                        std::numeric_limits<float>::lowest()};

  for (float angle_offset = -Deg2Rad(FLAGS_csm_angle_max); angle_offset < Deg2Rad(FLAGS_csm_angle_max); angle_offset += Deg2Rad(FLAGS_csm_angle_step)) {

    float log_likelihood_odom_angle = odom_angle_pdf_const - 0.5 * pow(abs(angle_offset)/FLAGS_sd_odom_angle, 2);
    float angle = AngleMod(rel_odom_angle + angle_offset);
    Eigen::Rotation2Df rotation(angle);
    
    vector<Vector2f> rotated_point_cloud = point_cloud;

    for (auto& point : rotated_point_cloud) {
      // Relative to previous odometry
      point = rotation * point;
    }

    for (float x_offset = -FLAGS_csm_transl_max; angle < FLAGS_csm_transl_max; angle += FLAGS_csm_transl_step) {

      float log_likelihood_odom_x = odom_x_pdf_const - 0.5 * pow(abs(x_offset)/FLAGS_sd_odom_x, 2);
      float x = rel_odom_loc[0] + x_offset;

      for (float y_offset = -FLAGS_csm_transl_max; angle < FLAGS_csm_transl_max; angle += FLAGS_csm_transl_step) {

        float log_likelihood_odom_y = odom_y_pdf_const - 0.5 * pow(abs(y_offset)/FLAGS_sd_odom_y, 2);
        float y = rel_odom_loc[1] + y_offset;

        // Compute odom likelihood
        float odom_log_likelihood = log_likelihood_odom_angle + log_likelihood_odom_x + log_likelihood_odom_y;

        // Relative to previous odometry
        Eigen::Vector2f loc = Eigen::Vector2f(x, y);

        float raster_score = 0;
        for (auto& point : rotated_point_cloud) {
          // Relative to previous odometry
          Eigen::Vector2f tranformed_point = point + loc;
          Eigen::Vector2f raster_loc = (tranformed_point.array() + FLAGS_raster_map_dist)/FLAGS_raster_pixel_dist;
          raster_score += raster((int)raster_loc[0], (int)raster_loc[1]);
        }

        // Evaluate log likelihood of each
        // // Raster log likelihood PLUS odometry log likelihood

        float log_likelihood = odom_log_likelihood + raster_score;
        if (log_likelihood > results.log_likelihood) {
          results.angle = angle;
          results.loc = loc;
          results.log_likelihood = log_likelihood;
        }

      }
    }
  }

  // Select pose with max log likelihood
  
  return results;
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  if (!odom_initialized_) {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
    return;
  }
  // Keep track of odometry to estimate how far the robot has moved between 
  // poses.

  current_odom_angle_ = odom_angle;
  current_odom_loc_ = odom_loc;
}

vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.

  // Iterate through all of the saved point clouds
  for (auto& pose : previous_poses_) {
    for (auto& point : pose.point_cloud) {
      map.push_back(point);
    }
  }
  // Add each one to the mega point cloud
  
  // That's it
  return map;
}

}  // namespace slam
