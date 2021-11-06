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
DEFINE_double(min_odom_angle_diff, 20, "Minimum Odom rotation diff to create a new pose");

DEFINE_double(sd_laser, 0.5, "Std dev of laser scan");
DEFINE_double(sd_laser_fine, 0.05, "Std dev of laser scan");
DEFINE_double(sd_odom_x, 1.0, "Std dev of odometry in x direction");
DEFINE_double(sd_odom_y, 1.0, "Std dev of odometry in x direction");
DEFINE_double(sd_odom_angle, 30.0, "Std dev of odometry in x direction");

DEFINE_double(raster_map_dist, 12.0, "Maximum distance of x & y axes in the rasterized map");
DEFINE_double(raster_pixel_dist, 0.01, "Size of each pixel in the raster map");
DEFINE_double(csm_transl_max, 0.8, "Max translation for CSM");
DEFINE_double(csm_angle_max, 30, "Max rotation for CSM");
DEFINE_double(csm_transl_step, 0.1, "Translation step size for CSM");
DEFINE_double(csm_transl_fine_step, 0.02, "Translation step size for CSM");
DEFINE_double(csm_angle_step, 1, "Rotation step size for CSM");
DEFINE_double(csm_angle_fine_step, 0.05, "Rotation step size for CSM");
DEFINE_int32(map_scan_mod, 4, "The module of the number of point used during create map");
DEFINE_int32(csm_scan_mod, 1, "The module of the number of point used during CSM");


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
using math_util::Clamp;

namespace slam {

SLAM::SLAM() :
    num_pixels_((int)((2. * FLAGS_raster_map_dist)/FLAGS_raster_pixel_dist)),
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    current_odom_loc_(0, 0),
    current_odom_angle_(0),
    pose_initialized_(false),
    odom_counter_(16) {}

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
    if (range > range_min && range < range_max) {
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
    prev_poses_.push_back(PoseData{Eigen::Vector2f(0,0), 0});
    prev_point_cloud_ = point_cloud;
    pose_initialized_ = true;

    std::cout << "Pose initialized!\n\n";
  } else {

    float angle_diff = AngleDiff(current_odom_angle_, prev_odom_angle_);
    Eigen::Vector2f loc_diff = current_odom_loc_ - prev_odom_loc_;

    if (odom_initialized_ && (loc_diff.norm() > FLAGS_min_odom_loc_diff || RadToDeg(abs(angle_diff)) > FLAGS_min_odom_angle_diff)) {
      std::cout << "Loc Diff: " << loc_diff.norm() << "\n";
      std::cout << "Angle Diff: " << RadToDeg(angle_diff) << "\n";
      std::cout << "Using laser scan!\n";

      // Compute best pose
      std::vector<Eigen::Vector2f> point_cloud = ScanToPointCloud(ranges, range_min, range_max, angle_min, angle_max);
      std::cout << "Created point cloud!\n";
      Eigen::MatrixXf raster = RasterizePointCloud(prev_point_cloud_, false);
      Eigen::MatrixXf raster_fine = RasterizePointCloud(prev_point_cloud_, true);
      std::cout << "Created raster map!\n";
      SLAM::CsmData csm_data = CSM(point_cloud, raster, raster_fine);
      std::cout << "Finished CSM!\n\n";

      // Convert to SLAM global frame from previous
      PoseData pose;
      Eigen::Rotation2Df rotation_pose(prev_poses_.back().angle);
      pose.angle = AngleMod(csm_data.angle + prev_poses_.back().angle);
      pose.loc = rotation_pose * csm_data.loc + prev_poses_.back().loc;

      prev_point_cloud_ = point_cloud;

      Eigen::Rotation2Df rotation_new_pose(pose.angle);

      // Add new pose and point cloud to map
      prev_poses_.push_back(pose);
      for (uint i = 0; i < point_cloud.size(); i += FLAGS_map_scan_mod) {
        Eigen::Vector2f new_point = rotation_new_pose * point_cloud[i] + pose.loc;
        map_.push_back(new_point);
      }

      // Update Odom
      prev_odom_angle_ = current_odom_angle_;
      prev_odom_loc_ = current_odom_loc_;
    }
  }

}

Eigen::MatrixXf SLAM::RasterizePointCloud(const vector<Eigen::Vector2f> point_cloud, bool fine_sd) {
  
  float sd_laser;

  if (fine_sd) {
    sd_laser = FLAGS_sd_laser_fine;
  } else {
    sd_laser = FLAGS_sd_laser;
  }

  // float min_value = std::numeric_limits<float>::lowest();
  float min_value = -10;
  Eigen::MatrixXf raster = Eigen::MatrixXf::Constant(num_pixels_, num_pixels_, min_value);

  // Compute max log likelihood of every point in the map (max from each point in point cloud)
  static const float gaussian_pdf_const = -log(sd_laser) - 0.5 * log(2 * M_PI);
  for (auto& pt : point_cloud) {
    Eigen::Vector2f raster_loc = (pt.array() + FLAGS_raster_map_dist)/FLAGS_raster_pixel_dist;
    // Clamp
    uint index_x_min = (uint) Clamp((int)raster_loc[0] - 20, 0, (int) num_pixels_ - 1);
    uint index_x_max = (uint) Clamp((int)raster_loc[0] + 20, 0, (int) num_pixels_ - 1);
    uint index_y_min = (uint) Clamp((int)raster_loc[1] - 20, 0, (int) num_pixels_ - 1);
    uint index_y_max = (uint) Clamp((int)raster_loc[1] + 20, 0, (int) num_pixels_ - 1);

    for (uint x_ind = index_x_min; x_ind <= index_x_max; x_ind++) {
      for (uint y_ind = index_y_min; y_ind <= index_y_max; y_ind++) {
        
        Eigen::Vector2f loc = FLAGS_raster_pixel_dist * Eigen::Vector2f(x_ind, y_ind).array() - FLAGS_raster_map_dist;
        float dist = (loc - pt).norm();

        // Log of gaussian pdf
        float log_likelihood = gaussian_pdf_const - 0.5 * pow(dist/sd_laser, 2);
        
        raster(x_ind,y_ind) = std::max(raster(x_ind, y_ind), log_likelihood);
      }
    }

  }

  return raster;
}

SLAM::CsmData SLAM::CSM_Search(std::vector<Eigen::Vector2f> sampled_point_cloud, Eigen::MatrixXf raster,
                 float angle_offset_max, float angle_offset_step, float angle_est, 
                 float transl_offset_max, float transl_offset_step, Eigen::Vector2f transl_est) {
  
  SLAM::CsmData results = SLAM::CsmData{transl_est,
                                        angle_est,
                                        std::numeric_limits<float>::lowest()};
  
  // These are constants for the log gaussian pdf
  static const float odom_angle_pdf_const = -log(FLAGS_sd_odom_angle) - 0.5 * log(2 * M_PI);
  static const float odom_x_pdf_const = -log(FLAGS_sd_odom_x) - 0.5 * log(2 * M_PI);
  static const float odom_y_pdf_const = -log(FLAGS_sd_odom_y) - 0.5 * log(2 * M_PI);

  static const float sd_odom_angle = DegToRad(FLAGS_sd_odom_angle);

  for (float angle_offset = -angle_offset_max; angle_offset < angle_offset_max; angle_offset += angle_offset_step) {

    float log_likelihood_odom_angle = odom_angle_pdf_const - 0.5 * pow(abs(angle_offset)/sd_odom_angle, 2);
    float angle = AngleMod(angle_est + angle_offset);
    Eigen::Rotation2Df rotation(angle);
    
    vector<Vector2f> rotated_point_cloud = sampled_point_cloud;

    for (auto& point : rotated_point_cloud) {
      // Relative to previous odometry
      point = rotation * point;
    }

    for (float x_offset = -transl_offset_max; x_offset < transl_offset_max; x_offset += transl_offset_step) {

      float log_likelihood_odom_x = odom_x_pdf_const - 0.5 * pow(abs(x_offset)/FLAGS_sd_odom_x, 2);
      float x = transl_est[0] + x_offset;

      for (float y_offset = -transl_offset_max; y_offset < transl_offset_max; y_offset += transl_offset_step) {

        float log_likelihood_odom_y = odom_y_pdf_const - 0.5 * pow(abs(y_offset)/FLAGS_sd_odom_y, 2);
        float y = transl_est[1] + y_offset;

        // Compute odom likelihood
        float odom_log_likelihood = log_likelihood_odom_angle + log_likelihood_odom_x + log_likelihood_odom_y;

        // Relative to previous odometry
        Eigen::Vector2f loc = Eigen::Vector2f(x, y);

        float raster_score = 0;
        for (auto& point : rotated_point_cloud) {
          // Relative to previous odometry
          Eigen::Vector2f tranformed_point = point + loc;
          Eigen::Vector2f raster_loc = (tranformed_point.array() + FLAGS_raster_map_dist)/FLAGS_raster_pixel_dist;

          // Clamp
          int index_x = std::max(std::min((int)raster_loc[0], (int) num_pixels_ - 1), 0);
          int index_y = std::max(std::min((int)raster_loc[1], (int) num_pixels_ - 1), 0);

          raster_score += raster(index_x, index_y);
        }

        // Evaluate log likelihood of each
        // Raster log likelihood PLUS odometry log likelihood

        float log_likelihood = odom_log_likelihood + raster_score;
        if (log_likelihood > results.log_likelihood) {
          //std::cout << log_likelihood << " Angle: " << RadToDeg(angle) << " Loc: " << loc.transpose() << "\n";
          results.angle = angle;
          results.loc = loc;
          results.log_likelihood = log_likelihood;
        }
      }
    }
  }

  return results;
}

SLAM::CsmData SLAM::CSM(const vector<Eigen::Vector2f> point_cloud, Eigen::MatrixXf raster, Eigen::MatrixXf raster_fine) {
  
  // Iterate through angle, dx, and dy
  // Center the search around our odomety position

  // Reduce Number of points in our point cloud
  vector<Vector2f> sampled_point_cloud;//(point_cloud.size()/FLAGS_csm_scan_mod);

  for (uint i = 0; i < point_cloud.size(); i += FLAGS_csm_scan_mod) {
    sampled_point_cloud.push_back(point_cloud[i]);
  }

  const Eigen::Rotation2Df rotation_odom(-prev_odom_angle_);
  Eigen::Vector2f rel_odom_loc = rotation_odom * (current_odom_loc_ - prev_odom_loc_);
  float rel_odom_angle = AngleDiff(current_odom_angle_, prev_odom_angle_);


  static const float csm_angle_max = DegToRad(FLAGS_csm_angle_max);
  static const float csm_angle_step = DegToRad(FLAGS_csm_angle_step);

  // for (float angle_offset = -csm_angle_max; angle_offset < csm_angle_max; angle_offset += csm_angle_step) {

  //   float log_likelihood_odom_angle = odom_angle_pdf_const - 0.5 * pow(abs(angle_offset)/sd_odom_angle, 2);
  //   float angle = AngleMod(rel_odom_angle + angle_offset);
  //   Eigen::Rotation2Df rotation(angle);
    
  //   vector<Vector2f> rotated_point_cloud = sampled_point_cloud;

  //   for (auto& point : rotated_point_cloud) {
  //     // Relative to previous odometry
  //     point = rotation * point;
  //   }

  //   for (float x_offset = -FLAGS_csm_transl_max; x_offset < FLAGS_csm_transl_max; x_offset += FLAGS_csm_transl_step) {

  //     float log_likelihood_odom_x = odom_x_pdf_const - 0.5 * pow(abs(x_offset)/FLAGS_sd_odom_x, 2);
  //     float x = rel_odom_loc[0] + x_offset;

  //     for (float y_offset = -FLAGS_csm_transl_max; y_offset < FLAGS_csm_transl_max; y_offset += FLAGS_csm_transl_step) {

  //       float log_likelihood_odom_y = odom_y_pdf_const - 0.5 * pow(abs(y_offset)/FLAGS_sd_odom_y, 2);
  //       float y = rel_odom_loc[1] + y_offset;

  //       // Compute odom likelihood
  //       float odom_log_likelihood = log_likelihood_odom_angle + log_likelihood_odom_x + log_likelihood_odom_y;

  //       // Relative to previous odometry
  //       Eigen::Vector2f loc = Eigen::Vector2f(x, y);

  //       float raster_score = 0;
  //       for (auto& point : rotated_point_cloud) {
  //         // Relative to previous odometry
  //         Eigen::Vector2f tranformed_point = point + loc;
  //         Eigen::Vector2f raster_loc = (tranformed_point.array() + FLAGS_raster_map_dist)/FLAGS_raster_pixel_dist;

  //         // Clamp
  //         int index_x = std::max(std::min((int)raster_loc[0], (int) num_pixels_ - 1), 0);
  //         int index_y = std::max(std::min((int)raster_loc[1], (int) num_pixels_ - 1), 0);

  //         raster_score += raster(index_x, index_y);
  //       }

  //       // Evaluate log likelihood of each
  //       // Raster log likelihood PLUS odometry log likelihood

  //       float log_likelihood = odom_log_likelihood + raster_score;
  //       if (log_likelihood > results.log_likelihood) {
  //         //std::cout << log_likelihood << " Angle: " << RadToDeg(angle) << " Loc: " << loc.transpose() << "\n";
  //         results.angle = angle;
  //         results.loc = loc;
  //         results.log_likelihood = log_likelihood;
  //       }

  //     }
  //   }
  // }

  SLAM::CsmData results = CSM_Search(sampled_point_cloud, raster,
                 csm_angle_max, csm_angle_step, rel_odom_angle, 
                 FLAGS_csm_transl_max, FLAGS_csm_transl_step, rel_odom_loc);

  // Fine CSM
  static const float csm_angle_fine_max = 2*DegToRad(FLAGS_csm_angle_step);
  static const float csm_angle_fine_step = DegToRad(FLAGS_csm_angle_fine_step);
  static const float csm_transl_fine_max = 2*FLAGS_csm_transl_step;
  static const float csm_transl_fine_step = FLAGS_csm_transl_step;
  // for (float angle_offset = -2*csm_angle_fine_max; angle_offset < 2*csm_angle_fine_max; angle_offset += csm_angle_fine_step) {

  //   float log_likelihood_odom_angle = odom_angle_pdf_const - 0.5 * pow(abs(angle_offset)/sd_odom_angle, 2);
  //   float angle = AngleMod(angle_coarse + angle_offset);
  //   Eigen::Rotation2Df rotation(angle);
    
  //   vector<Vector2f> rotated_point_cloud = sampled_point_cloud;

  //   for (auto& point : rotated_point_cloud) {
  //     // Relative to previous odometry
  //     point = rotation * point;
  //   }

  //   for (float x_offset = -2*FLAGS_csm_transl_step; x_offset < 2*FLAGS_csm_transl_step; x_offset += FLAGS_csm_transl_fine_step) {

  //     float log_likelihood_odom_x = odom_x_pdf_const - 0.5 * pow(abs(x_offset)/FLAGS_sd_odom_x, 2);
  //     float x = x_coarse + x_offset;

  //     for (float y_offset = -2*FLAGS_csm_transl_step; y_offset < 2*FLAGS_csm_transl_step; y_offset += FLAGS_csm_transl_fine_step) {

  //       float log_likelihood_odom_y = odom_y_pdf_const - 0.5 * pow(abs(y_offset)/FLAGS_sd_odom_y, 2);
  //       float y = y_coarse + y_offset;

  //       // Compute odom likelihood
  //       float odom_log_likelihood = log_likelihood_odom_angle + log_likelihood_odom_x + log_likelihood_odom_y;

  //       // Relative to previous odometry
  //       Eigen::Vector2f loc = Eigen::Vector2f(x, y);

  //       float raster_score = 0;
  //       for (auto& point : rotated_point_cloud) {
  //         // Relative to previous odometry
  //         Eigen::Vector2f tranformed_point = point + loc;
  //         Eigen::Vector2f raster_loc = (tranformed_point.array() + FLAGS_raster_map_dist)/FLAGS_raster_pixel_dist;

  //         // Clamp
  //         int index_x = std::max(std::min((int)raster_loc[0], (int) num_pixels_ - 1), 0);
  //         int index_y = std::max(std::min((int)raster_loc[1], (int) num_pixels_ - 1), 0);

  //         raster_score += raster(index_x, index_y);
  //       }

  //       // Evaluate log likelihood of each
  //       // Raster log likelihood PLUS odometry log likelihood

  //       float log_likelihood = odom_log_likelihood + raster_score;
  //       if (log_likelihood > results.log_likelihood) {
  //         //std::cout << log_likelihood << " Angle: " << RadToDeg(angle) << " Loc: " << loc.transpose() << "\n";
  //         results.angle = angle;
  //         results.loc = loc;
  //         results.log_likelihood = log_likelihood;
  //       }

  //     }
  //   }
  // }

  results = CSM_Search(sampled_point_cloud, raster_fine,
                csm_angle_fine_max, csm_angle_fine_step, results.angle, 
                csm_transl_fine_max, csm_transl_fine_step, results.loc);

  // Select pose with max log likelihood
  
  return results;
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  if (!odom_initialized_ && odom_counter_ <= 0) {
    std::cout << "Initializing odom! \n";
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
    return;
  } else {
    odom_counter_--;
  }

  // Keep track of odometry to estimate how far the robot has moved between 
  // poses.

  current_odom_angle_ = odom_angle;
  current_odom_loc_ = odom_loc;
}

vector<Vector2f> SLAM::GetMap() {
  // That's it
  return map_;
}

}  // namespace slam
