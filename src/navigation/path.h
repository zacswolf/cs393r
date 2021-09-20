
#include <vector>

#include "eigen3/Eigen/Dense"

using namespace navigation;
using Eigen::Vector2f;

DEFINE_double(min_clearance, .02, "The min clearance, this accounts for lidar noise");
DEFINE_int32(rate_path, 1, "The min clearance");

DEFINE_double(fpl_mult, 1.2, "The free path length multiplier");
DEFINE_double(curvature_mult, 1, "The curvature multiplier");
DEFINE_double(side_mult, 1, "The side change multiplier");

DEFINE_double(clearance_mult1, 0, "The d-1 poly clearance multiplier");
DEFINE_double(clearance_mult2, .000355, "The d-2 poly clearance multiplier");




#ifndef PATH_H
#define PATH_H

class Path {
  public:
  float curvature;
  float radius;
  int side;
  float clearance;
  float free_path_length;

  float arc_angle;
  Eigen::Vector2f car_pos;

  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  Eigen::Vector2f clearance_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  Path(float curvature) : curvature(curvature) {
    this->radius = (curvature == 0.)? 0. : 1./abs(curvature);
    this->side = (0. < curvature) - (curvature < 0.);
  }

  float point_to_path_dist(const Vector2f& goal_point) {
    if (this->curvature == 0) {
      return goal_point[1];
    } else {
      float arc_angle_to_point = fmod(atan2(goal_point[0], -this->side * goal_point[1] + this->radius) + 2*M_PI, 2*M_PI);
      if (arc_angle_to_point < this->arc_angle) {
        Vector2f center_point = Vector2f(0., this->radius * this->side);
        return (goal_point-center_point).norm();
      } else {
        // return min((goal_point-this.car_pos).norm(), goal_point.norm());

        // I dont think we should project to the current position as that is useless 
        // when comparing paths and the point is behind us. I think we'd want to "turn around".
        return (goal_point-this->car_pos).norm(); 
      }
    }
  }

  void add_collision_data(float free_path_length, Vector2f closest_point, float clearance, Vector2f clearance_point) {
    this->free_path_length = free_path_length;
    this->closest_point = closest_point;
    // TODO: Add more to this 
    this->arc_angle = free_path_length * abs(this->curvature);
    this->car_pos = this->radius * Vector2f(sin(this->arc_angle), this->side * (cos(this->arc_angle) - 1));
    this->clearance = clearance;
    this->clearance_point = clearance_point;
  }

  float rate_path(const Vector2f& goal_point, Vector2f& closest_barrier_point, float previous_curv) {
    if (closest_barrier_point[0] > -FLAGS_del_length && closest_barrier_point[0] < FLAGS_length + FLAGS_del_length && abs(closest_barrier_point[1]) < FLAGS_width/2 + FLAGS_del_width) {
      printf("Collision!\n");
    }

    float clearance_to_side = std::min(0., this->clearance - FLAGS_width/2 - FLAGS_del_width);
    float clearance_penalty;
    if (clearance_to_side <= FLAGS_min_clearance) {
      // Dont use this path as lidar noise could cause a collision with the safety margin
      clearance_penalty = 100;
    } else {
      clearance_penalty = (FLAGS_clearance_mult2 / pow(clearance_to_side, 2)) + (FLAGS_clearance_mult1 / clearance_to_side);
    }

    int previous_side = (0 < previous_curv) - (previous_curv < 0);
    int side_penalty = previous_side ^= this->side;

    float neg_free_path_length_norm = (-1./8) * this->free_path_length;

    if (FLAGS_rate_path == 0) {
      return FLAGS_fpl_mult*(neg_free_path_length_norm) + FLAGS_curvature_mult*(1./800)*abs(this->curvature) + clearance_penalty + FLAGS_side_mult*(1./160)*side_penalty;
    } else {
      return (-1./8)*this->free_path_length + (1./800)*abs(this->curvature) + (100./8)*clearance_penalty + (1./160)*side_penalty;
    }
  }
};

#endif  // PATH_H
