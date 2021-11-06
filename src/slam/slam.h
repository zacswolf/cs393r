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
\file    slam.h
\brief   SLAM Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_


namespace slam {


class SLAM {

 public:
  // Default Constructor.
  SLAM();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Get latest map.
  std::vector<Eigen::Vector2f> GetMap();

  // Get latest robot pose.
  void GetPose(Eigen::Vector2f* loc, float* angle) const;

  // Our functions

  // Convert scan to point cloud
  std::vector<Eigen::Vector2f> ScanToPointCloud(
                    const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  // Rasterize into our map
  // MatrixXf is of dimention raster_map_dist/raster_dist
  Eigen::MatrixXf RasterizePointCloud(const std::vector<Eigen::Vector2f> point_cloud);


  struct CsmData {
    Eigen::Vector2f loc;
    float angle;
    float log_likelihood;
  };

  // Correlative Scan Matching
  CsmData CSM(const std::vector<Eigen::Vector2f> point_cloud, Eigen::MatrixXf raster);

 private:

  const uint num_pixels_;

  CsmData CSM_Search(std::vector<Eigen::Vector2f> sampled_point_cloud, Eigen::MatrixXf raster,
                 float angle_offset_max, float angle_offset_step, float angle_est, 
                 float transl_offset_max, float transl_offset_step, Eigen::Vector2f transl_est);

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;

  // Current odometry-reported locations.
  Eigen::Vector2f current_odom_loc_;
  float current_odom_angle_;

  // Previous pose
  bool pose_initialized_;
  
  struct PoseData {
    Eigen::Vector2f loc;
    float angle;
  };

  std::vector<Eigen::Vector2f> map_;

  std::vector<PoseData> prev_poses_;
  std::vector<Eigen::Vector2f> prev_point_cloud_;

  int odom_counter_;

};
}  // namespace slam

#endif   // SRC_SLAM_H_
