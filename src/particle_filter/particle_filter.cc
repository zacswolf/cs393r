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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
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
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"


#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using std::numeric_limits;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");

DEFINE_double(sd_predict_x, .1, "Std Dev of local x error");
DEFINE_double(sd_predict_y, .05, "Std Dev of local y error");
DEFINE_double(sd_predict_angle, 0.1*M_PI/180, "Std Dev of angle error");

DEFINE_double(gamma, 1./20., "Gamma: LIDAR correlation coefficient");
DEFINE_double(sd_laser, 0.002, "Std Dev");

DEFINE_int32(loc_algo, 0, "Localization algorithm (0 = mode, 1 = mean)");

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {
      particles_.resize(FLAGS_num_particles);
      resample_counter_ = 0;
    }

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  scan.resize(num_ranges);

  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.
  const Eigen::Rotation2Df rotation(angle);
  const Vector2f kLaserLoc(0.2, 0); // Relative to car
  const Vector2f kLaserLoc_m = loc + rotation * kLaserLoc; // relative to map

  vector<line2f> rays(num_ranges); // Relative to map
  float ray_angle = angle_min; // Relative to car
  const float ray_increment = (angle_max - angle_min)/(num_ranges-1);

  // Populate rays with the lines for each lidar measurement
  for (int i = 0; i < num_ranges; i++) {
    // TODO: Check math, especially the sign of angle
    // TODO: Clean up

    // Relative to the car
    Vector2f start = Vector2f(range_min * cos(ray_angle), range_min * sin(ray_angle)) + kLaserLoc;
    Vector2f end = Vector2f(range_max * cos(ray_angle), range_max * sin(ray_angle)) + kLaserLoc;

    // Relative to the map
    start = loc + rotation * start;
    end = loc + rotation * end;

    rays[i] = line2f(start, end);

    ray_angle += ray_increment;
  }
  

  // Find closest intersection point for each ray
  for (size_t i = 0; i < rays.size(); i++) {
    const line2f ray_line = rays[i];

    Vector2f closest_point;
    double closest_point_dist = 100.; //numeric_limits<float>::max();

    // Compare each ray to each line in the map
    for (size_t j = 0; j < map_.lines.size(); j++) {
      const line2f map_line = map_.lines[j];

      Vector2f intersection_point; 
      const bool intersects = ray_line.Intersection(map_line, &intersection_point);

      // rel to map -> rel to lidar
      intersection_point = intersection_point - kLaserLoc_m;
      if (intersects && intersection_point.norm() < closest_point_dist) {
        closest_point = intersection_point;
        closest_point_dist = intersection_point.norm();
      }
    }
    scan[i] = closest_point;
  }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.

  const int num_ranges = ranges.size(); //TODO: Verify that the size of this doesnt change
  assert(num_ranges == 1081);

  double max_log_weight = -1000000.;numeric_limits<double>::lowest();

  // Determine log weights
  for (auto& particle : particles_) {
    vector<Vector2f> scan;

    GetPredictedPointCloud(particle.loc, particle.angle, num_ranges, range_min, range_max, angle_min, angle_max, &scan);
    
    // Convert GetPredictedPointCloud to ranges
    vector<double> scan_range(scan.size());
    for (uint i = 0; i < scan.size(); i++) {
      scan_range[i] = scan[i].norm();
    }

    // TODO: robust weighting
    // Compare the scan_range to ranges
    double sum = 0.;
    uint num_used_ranges = 0;
    for (uint i = 0; i < scan.size(); i++) {
      if (scan_range[i] < range_max && scan_range[i] > range_min && ranges[i] < range_max && ranges[i] > range_min) {
        sum += pow((scan_range[i] - ranges[i]) / FLAGS_sd_laser, 2.);
        num_used_ranges++;
      } // TODO: Maybe add case where scan_range valid xor range valid
    }

    // Note: in inconsistent state 
    particle.weight = -FLAGS_gamma * sum;
    std::cout << particle.weight << " ";
    // Find max particle log weight
    if (particle.weight > max_log_weight) {
      max_log_weight = particle.weight;
    }
  }

  std::cout << "\n";

  // Convert log weights to non-absolute weights
  double weight_sum = 0.; // Only used if FLAGS_loc_algo != 0
  for (auto& particle : particles_) {
    particle.weight -= max_log_weight;
    particle.weight = exp(particle.weight);
    weight_sum += particle.weight;
  }

  // Normalize weights
  if (FLAGS_loc_algo) {
    for (auto& particle : particles_) {
      particle.weight /= weight_sum;
    }
  }
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  // vector<Particle> new_particles;
  // During resampling: 
  //    new_particles.push_back(...)
  // After resampling:
  // particles_ = new_particles;

  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:

  vector<Particle> new_particles;
  // (1) prefix sum the weights: ps = [0, w_1, ..., sum (0<=i<N) v_i]
  // (2) sample rand (r) from 0, ps[-1] 
  // (3) BS ps for i where ps[i] > r and ps[i-1] < r
  // (4) goto (2) [alt: low-variance sampling]

  // Compute prefix sum
  vector<double> prefix_sum(particles_.size() + 1);
  prefix_sum[0] = 0.;
  for (uint i = 1; i < prefix_sum.size(); i++){
    prefix_sum[i] = prefix_sum[i-1] + particles_[i-1].weight;
  }

  // Random sample
  for (uint i = 0; i < particles_.size(); i++) {
    double rand = rng_.UniformRandom(0, prefix_sum[particles_.size() + 1]);

    // Find index of prefix_sum where ps[j-1] < r && ps[j] > r
    // TODO: Binary Search?
    for (uint j = 1; j < prefix_sum.size(); j++) {
      if (prefix_sum[j] >= rand) {
        new_particles.push_back(Particle{Vector2f(particles_[j-1].loc[0], particles_[j-1].loc[1]), particles_[j-1].angle, 1./particles_.size()});
        // new_particles.push_back(particles_[j-1]);
        // new_particles[i].weight = 1./particles_.size();
        break;
      }
    }
  }

  assert(new_particles.size() == particles_.size());
  
  // Update particles
  particles_ = new_particles;

}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.

  Update(ranges, range_min, range_max, angle_min, angle_max, nullptr);

  if (resample_counter_ == 0) {
    Resample();
  }
  resample_counter_ = (resample_counter_ + 1) % 15;
}

void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.


  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:

  if (!odom_initialized_){
    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
    odom_initialized_ = true;
  }

  Vector2f delt_loc = odom_loc - prev_odom_loc_;
  float delt_angle = odom_angle - prev_odom_angle_;

  Eigen::Rotation2Df r1(-prev_odom_angle_); // rotate odom -> local_prev

  for (auto& particle : particles_) {
    Eigen::Rotation2Df r2(particle.angle); // rotate local_prev -> map
    particle.loc += r2*(r1*delt_loc + Vector2f(rng_.Gaussian(0., FLAGS_sd_predict_x), rng_.Gaussian(0., FLAGS_sd_predict_y)));
    particle.angle += delt_angle + rng_.Gaussian(0., FLAGS_sd_predict_angle);
  }

  prev_odom_angle_ = odom_angle;
  prev_odom_loc_ = odom_loc;
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);


  particles_.resize(FLAGS_num_particles);
  for (auto& particle : particles_) {
    Vector2f loc_offset = Vector2f(rng_.UniformRandom(-0.005, 0.005), rng_.UniformRandom(-0.005, 0.005));
    float ang_offset = rng_.UniformRandom(-M_PI/80., M_PI/80.);
    particle.loc = loc + loc_offset;
    particle.angle = angle + ang_offset;
    particle.weight = 1./(double)FLAGS_num_particles;
  }

  odom_initialized_ = false;
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  
  if (!FLAGS_loc_algo) {
    // MODE: ie return particle with highest weight
    Particle mode_particle{Vector2f(0., 0.), 0., 0.};
    for (auto& particle : particles_) {
      if (particle.weight > mode_particle.weight) {
        mode_particle = particle;
      }
      std::cout << particle.weight << " ";
    }
    std::cout << "\n\n";
    loc = mode_particle.loc;
    angle = mode_particle.angle;
  } else {
    // MEAN: ie return weighted average of loc and angle
    // NOTE: Requires weights to be normalized
    loc = Vector2f(0., 0.);
    Vector2f angle_iq = Vector2f(0., 0.);

    for (uint i = 0; i < particles_.size(); i++) {
      loc += particles_[i].loc * particles_[i].weight;
      angle_iq = angle_iq + Vector2f(cos(particles_[i].angle), sin(particles_[i].angle)) * particles_[i].weight;
    }

    angle = atan2(angle_iq[1], angle_iq[0]);
  }
}

}  // namespace particle_filter
