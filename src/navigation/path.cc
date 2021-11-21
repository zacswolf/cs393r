
#include <vector>

#include "eigen3/Eigen/Dense"
#include "visualization/visualization.h"
#include "gflags/gflags.h"
#include "path.h"
#include "shared/math/math_util.h"


using Eigen::Vector2f;
using namespace math_util;

DEFINE_double(min_clearance, .1, "The min clearance, this accounts for lidar noise");
DEFINE_int32(rate_path, 0, "The rate path algo to use");

DEFINE_double(fpl_mult, 1.2, "The free path length multiplier");
DEFINE_double(curvature_mult, 1, "The curvature multiplier");
DEFINE_double(side_mult, 1, "The side change multiplier");

DEFINE_double(clearance_mult1, 0, "The d-1 poly clearance multiplier");
DEFINE_double(clearance_mult2, .000355, "The d-2 poly clearance multiplier");

  
Path::Path(float curvature) : curvature(curvature), vehicle_radius_min(0), 
                        vehicle_radius_max(std::numeric_limits<float>::max()) {
  this->radius = (curvature == 0.)? 0. : 1./abs(curvature);
  this->side = (0. < curvature) - (curvature < 0.);
}

float Path::point_to_path_dist(const Vector2f& goal_point) {
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

void Path::add_collision_data(float free_path_length, Vector2f closest_point, float clearance, Vector2f clearance_point) {
  this->free_path_length = free_path_length;
  this->closest_point = closest_point;
  // TODO: Add more to this 
  this->arc_angle = free_path_length * abs(this->curvature);
  this->car_pos = this->radius * Vector2f(sin(this->arc_angle), this->side * (cos(this->arc_angle) - 1));
  this->clearance = clearance;
  this->clearance_point = clearance_point;
}

float Path::rate_path(const Vector2f& goal_point, float previous_curv) {
  
  //float clearance_to_side = std::min(0., this->clearance - FLAGS_width/2 - FLAGS_del_width);
  float clearance_to_side = this->clearance;
  float clearance_penalty;
  if (clearance_to_side <= FLAGS_min_clearance) {
    // Dont use this path as lidar noise could cause a collision with the safety margin
    clearance_penalty = 10000;
  } else {
    clearance_penalty = (FLAGS_clearance_mult2 / pow(clearance_to_side, 2)) + (FLAGS_clearance_mult1 / clearance_to_side);
  }

  // Angle and dist to goal point
  float angle_to_goal = atan2(goal_point[1], goal_point[0]);
  float dist_to_goal = goal_point.norm();
  float curvature_rating = point_to_path_dist(goal_point);
  float angle_rating = -this->curvature * angle_to_goal / (dist_to_goal+.0001);

  std::cout << "curvature_rating: " << curvature_rating << "; angle_rating: " << angle_rating << std::endl;

  float goal_point_loss = curvature_rating + 10*angle_rating;

  // If point is behind us, focus on not colliding
  if (abs(angle_to_goal) > 3.*M_PI/4.){
    goal_point_loss = 0.;
  }

  float neg_free_path_length_norm = -1 * this->free_path_length;
  return FLAGS_fpl_mult*(neg_free_path_length_norm) + goal_point_loss + clearance_penalty;
}

float Path::rate_path_alt(const Vector2f& goal_point, float previous_curv) {

  float fpl_cost = 0;
  float clearance_cost = 0;
  float goal_cost = 0;
  float angle_to_goal = atan2(goal_point[1], goal_point[0]);

  if (this->clearance < FLAGS_min_clearance) {
    clearance_cost = 1000 + 2./this->clearance;
  }

  if (this->free_path_length < 2.) {
    fpl_cost = 2000 + 1./this->free_path_length;
  }

  if (goal_point.norm() < this->free_path_length) {
    fpl_cost = 0;
  }

  // if (this->free_path_length > 4.) {
  //   // don't care
  //   fpl_cost = 0;
  //   clearance_cost = 0;
  // } else if (this->free_path_length > 2) {
  //   // start worry about clearance
  //   fpl_cost = 0;
  // } else {
  //   // avoid
  //   fpl_cost = ;
  // }

  goal_cost = pow(this->curvature - 2*angle_to_goal, 2); // range: -3.14 to 3.14

  return fpl_cost + clearance_cost + goal_cost;

}

void Path::visualize(amrl_msgs::VisualizationMsg& local_viz_msg_) {
  int side;
  float start_angle;
  float end_angle;
  Eigen::Vector2f center;

  if (this->curvature == 0) {
    visualization::DrawLine(Vector2f(0., 0.), Vector2f(this->free_path_length, 0.), 0xfca903, local_viz_msg_);
  } else {
    side = (0 < this->curvature) - (this->curvature < 0);
    start_angle = side * -M_PI/2;
    end_angle = start_angle + this->free_path_length * this->curvature;
    center = Vector2f(0, 1/this->curvature);
    
    if (side == 1) {
      start_angle = -side*M_PI/2;
      end_angle = start_angle + this->free_path_length * this->curvature;
    } else {
      start_angle = start_angle + this->free_path_length * this->curvature;
      end_angle = -side*M_PI/2;
    }

    visualization::DrawArc(center,
                            this->radius,
                            start_angle,
                            end_angle,
                            0xfca903,
                            local_viz_msg_);
  }
}
