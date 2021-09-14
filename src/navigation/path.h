
#include <vector>

#include "eigen3/Eigen/Dense"

using Eigen::Vector2f;

#ifndef PATH_H
#define PATH_H

class Path {
  public:
  float curvature;
  float radius;
  float clearance;
  float free_path_length;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  Path(float curvature) : curvature(curvature), radius(1/abs(curvature)) {}

  float point_to_path_dist(const Vector2f& goal_point) {
    return 0.;
  }

  void add_collision_data(float free_path_length, Vector2f closest_point) {
    this->free_path_length = free_path_length;
    this->closest_point = closest_point;
    // TODO: Add more to this 
  }

  float rate_path1(const Vector2f& goal_point) {
    // float distance;
    // if (curvature != 0) {
    //   Vector2f center_point = Vector2f(0., 1/curvature);

    // }
    return 0. * this->free_path_length + 1. * (goal_point - this->closest_point).norm();
  }
};

#endif  // PATH_H
