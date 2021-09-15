
#include <vector>

#include "eigen3/Eigen/Dense"

using Eigen::Vector2f;

DEFINE_double(max_acceleration, 4., "The max acceleration");
DEFINE_double(max_deceleration, 4., "The max deceleration");
DEFINE_double(max_speed, 1., "The max speed");

DEFINE_double(length, .4, "The wheel base of the robot");
DEFINE_double(width, .2, "The track width of the robot");
DEFINE_double(del_length, .1, "The length margin of the robot");
DEFINE_double(del_width, .1, "The width margin of the robot");
// DEFINE_double(safety_margin, .15, "The safety margin for the robot");

#ifndef COLLISION_H
#define COLLISION_H

namespace collision {

// Existance
bool is_inner_collision(double radius_pt, double radius_inner_back, double radius_inner_front) {
  return (radius_pt > radius_inner_back && radius_pt < radius_inner_front);
}

bool is_front_collision(double radius_pt, double radius_inner_front, double radius_outer_front) {
  return (radius_pt > radius_inner_front && radius_pt < radius_outer_front);
}

bool is_outer_collision(double radius_pt, double radius_outer_front, double radius_outer_back) {
  // TODO: Fix this
  return false; //(radius_pt > radius_outer_front && radius_pt < radius_outer_back);
}

bool is_straight_collision(Vector2f pt) {
  return (abs(pt[1]) <= FLAGS_width/2 + FLAGS_del_width);
}

// Distance to collision
double dist_to_collision_inner(double radius_car, int side, double radius_pt, Vector2f pt) {
  double theta_car = acos((radius_car - FLAGS_width/2 - FLAGS_del_width) / radius_pt);
  double theta_point = atan2(pt[0], radius_car - side*pt[1]);
  if (theta_point < 0) {
    theta_point = theta_point + 2*M_PI;
  }
  return std::max((theta_point - theta_car) * radius_car,0.);
}

double dist_to_collision_front(double radius_car, int side, double radius_pt, Vector2f pt) {
  double theta_car = asin((FLAGS_length + FLAGS_del_length) / radius_pt);
  double theta_point = atan2(pt[0], radius_car - side*pt[1]);
  if (theta_point < 0) {
    theta_point = theta_point + 2*M_PI;
  }
  return std::max((theta_point - theta_car) * radius_car,0.);
}

double dist_to_collision_outer(double radius_car, int side, double radius_pt, Vector2f pt) {
  double theta_car = acos((radius_car + FLAGS_width/2 + FLAGS_del_width) / radius_pt);
  double theta_point = atan2(pt[0], radius_car + side*pt[1]);
  if (theta_point < 0) {
    theta_point = theta_point + 2*M_PI;
  }
  return std::max((theta_point - theta_car) * radius_car,0.);
}

// big boi, returns max if no collison 
const double MAX_DIST = 8;
const double MAX_ANG = 1.5*M_PI;
double distance_to_collision(double curvature, Vector2f pt) {
  double radius_car = 1/abs(curvature);
  int side = (0 < curvature) - (curvature < 0);
  double radius_pt = (pt-side*Vector2f(0.,radius_car)).norm();

  double radius_left_back = sqrt(pow(radius_car - side*(FLAGS_width/2. + FLAGS_del_width), 2) + pow(FLAGS_del_length, 2));
  double radius_right_back = sqrt(pow(radius_car + side*(FLAGS_width/2. + FLAGS_del_width), 2) + pow(FLAGS_del_length, 2));
  double radius_left_front = sqrt(pow(radius_car - side*(FLAGS_width/2. + FLAGS_del_width), 2) + pow(FLAGS_length + FLAGS_del_length, 2));
  double radius_right_front = sqrt(pow(radius_car + side*(FLAGS_width/2. + FLAGS_del_width), 2) + pow(FLAGS_length + FLAGS_del_length, 2));

  if (pt[0] > -FLAGS_del_length && pt[0] < FLAGS_length+FLAGS_del_length && abs(pt[1]) < FLAGS_width/2 + FLAGS_del_width) {
    // Point inside car, ignore
    return 0;
  } else if (side == 0) {
    // Straight
    if (is_straight_collision(pt)) {
      return pt[0] - (FLAGS_length + FLAGS_del_length);
    }
    return MAX_DIST;
  } else {
    double radius_inner_back;
    double radius_inner_front;
    double radius_outer_back;
    double radius_outer_front;

    if (side == 1) {
      // Left
      radius_inner_back = radius_left_back;
      radius_inner_front = radius_left_front;
      radius_outer_back = radius_right_back;
      radius_outer_front = radius_right_front;
    } else { // (side == -1)
      // Right
      radius_inner_back = radius_right_back;
      radius_inner_front = radius_right_front;
      radius_outer_back = radius_left_back;
      radius_outer_front = radius_left_front;
    }

    if (is_inner_collision(radius_pt, radius_inner_back, radius_inner_front)) {
      return dist_to_collision_inner(radius_car, side, radius_pt, pt);
    } else if (is_front_collision(radius_pt, radius_inner_front, radius_outer_front)) {
      return dist_to_collision_front(radius_car, side, radius_pt, pt);
    } else if (is_outer_collision(radius_pt, radius_outer_front, radius_outer_back)) {
      return dist_to_collision_outer(radius_car, side, radius_pt, pt);
    } else {
      return std::min(MAX_ANG*radius_car, MAX_DIST);
    }
  }
}

}  // namespace navigation

#endif  // COLLISION_H
