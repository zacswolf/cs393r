
#include <vector>

#include "eigen3/Eigen/Dense"

using Eigen::Vector2f;

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
    this->radius = (curvature==0)? 0 : 1/abs(curvature);
    this->side = (0 < curvature) - (curvature < 0);
  }

  float point_to_path_dist(const Vector2f& goal_point) {
    if (this->curvature == 0) {
      return goal_point[1];
    } else {
      float arc_angle_to_point = fmod(atan2(goal_point[0], goal_point[1] - this->side * this->radius)+ 2*M_PI, 2*M_PI);
      if (arc_angle_to_point < this->arc_angle) {
        Vector2f center_point = Vector2f(0, this->radius * this->side);
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
    this->car_pos = Vector2f(this->radius * sin(this->arc_angle), this->side * this->radius * (cos(this->arc_angle) - 1));
    this->clearance = clearance;
    this->clearance_point = clearance_point;
  }

  float rate_path1(const Vector2f& goal_point, Vector2f& closest_barrier_point) {

    float barrier_penalty = 0;
    if (closest_barrier_point[0] > -FLAGS_del_length && closest_barrier_point[0] < FLAGS_length+FLAGS_del_length && abs(closest_barrier_point[1]) < FLAGS_width/2 + FLAGS_del_width) {
      barrier_penalty = this->curvature/closest_point[1];
      printf("Collision!");
    }

    float clearance_penalty = 0;
    if (this->clearance < FLAGS_width/2 + FLAGS_del_width + 0.2) {
      clearance_penalty = 1;
      //printf("Avoiding close clearance!");
    }

    return -this->free_path_length + 0.01*abs(this->curvature) + 100*clearance_penalty + 100*barrier_penalty;
    //return 300*this->free_path_length * (this->free_path_length < .03) + 0 * 1/this->free_path_length + 1. * this->point_to_path_dist(goal_point);
  }
};

#endif  // PATH_H
