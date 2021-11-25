
#include <vector>
#include "eigen3/Eigen/Dense"
#include "car.h"
#include "gflags/gflags.h"

using Eigen::Vector2f;


DEFINE_double(clearance_drop_off, .1, "The clearance drop off, use 0 for normal clearance");


// DEFINE_double(max_acceleration, 4., "The max acceleration");
// DEFINE_double(max_deceleration, 4., "The max deceleration");
// DEFINE_double(max_speed, 1., "The max speed");

// DEFINE_double(length, .32385, "The wheel base of the robot");
// DEFINE_double(width, .2286, "The track width of the robot");
// DEFINE_double(del_length, .088075 + .05, "The length margin of the robot");
// DEFINE_double(del_width, .0254 + .05, "The width margin of the robot");
// // DEFINE_double(safety_margin, .15, "The safety margin for the robot");

Car::Car(double max_acceleration, double max_deceleration, double max_speed, double length, double width, double del_length, double del_width, double safety_margin) : 
          max_acceleration(max_acceleration),
          max_deceleration(max_deceleration),
          max_speed(max_speed),
          length(length),
          width(width),
          del_length(del_length + safety_margin),
          del_width(del_length + safety_margin) {}

void Car::drawBoundingBox(amrl_msgs::VisualizationMsg& local_viz_msg_) {
  // Draw car bounding box
  Vector2f p_bl = Vector2f(-del_length, width/2 + del_width);
  Vector2f p_br = Vector2f(-del_length, -1*(width/2 + del_width));
  Vector2f p_fl = Vector2f(length + del_length, width/2 + del_width);
  Vector2f p_fr = Vector2f(length + del_length, -1*(width/2 + del_width));
  visualization::DrawLine(p_bl, p_fl, 0x00FFAA, local_viz_msg_);
  visualization::DrawLine(p_br, p_fr, 0x00FFAA, local_viz_msg_);
  visualization::DrawLine(p_fl, p_fr, 0x00FFAA, local_viz_msg_);
  visualization::DrawLine(p_bl, p_br, 0x00FFAA, local_viz_msg_);
}

////////////// COLLISION
// Existance
bool Car::is_inner_collision(double radius_pt, double radius_inner_back, double radius_inner_front) {
  return (radius_pt > radius_inner_back && radius_pt < radius_inner_front);
}

bool Car::is_front_collision(double radius_pt, double radius_inner_front, double radius_outer_front) {
  return (radius_pt > radius_inner_front && radius_pt < radius_outer_front) ;
}

bool Car::is_outer_collision(double radius_pt, double radius_outer_front, double radius_outer_back) {
  // TODO: Fix this
  return false; //(radius_pt > radius_outer_front && radius_pt < radius_outer_back);
}

bool Car::is_straight_collision(Vector2f pt) {
  return (abs(pt[1]) <= (width)/2 + del_width);
}

// Distance to collision
double Car::dist_to_collision_inner(double radius_car, int side, double radius_pt, Vector2f pt) {
  double theta_car = acos((radius_car - width/2 - del_width)/radius_pt);
  double theta_point = fmod(atan2(pt[0], radius_car - side*pt[1]) + 2*M_PI, 2*M_PI);
  return std::max(fmod(theta_point - theta_car + 2*M_PI, 2*M_PI) * radius_car,0.);
}

double Car::dist_to_collision_front(double radius_car, int side, double radius_pt, Vector2f pt, double curvature) {
  double theta_car = asin((length + del_length)/radius_pt);
  double theta_point = fmod(atan2(pt[0], radius_car - side*pt[1]) + 2*M_PI, 2*M_PI);
  return std::max(fmod(theta_point - theta_car + 2*M_PI, 2*M_PI) * radius_car,0.);
}

double Car::dist_to_collision_outer(double radius_car, int side, double radius_pt, Vector2f pt) {
  double theta_car = acos((radius_car + width/2 + del_width)/radius_pt);
  double theta_point = fmod(atan2(pt[0], radius_car - side*pt[1]) + 2*M_PI, 2*M_PI);
  return std::max(fmod(theta_point - theta_car + 2*M_PI, 2*M_PI) * radius_car, 0.);
}

// big boi, returns max if no collison 

double Car::distance_to_collision(Path& path, Vector2f pt) {
  const double MAX_DIST = 8;
  const double MAX_ANG = 1.5 * M_PI;
  double curvature = path.curvature;
  double radius_car = path.radius;

  int side = (0 < curvature) - (curvature < 0);
  double radius_pt = (pt-side*Vector2f(0.,radius_car)).norm();

  double radius_left_back = sqrt(pow(radius_car - side*(width/2. + del_width), 2) + pow(del_length, 2));
  double radius_right_back = sqrt(pow(radius_car + side*(width/2. + del_width), 2) + pow(del_length, 2));
  double radius_left_front = sqrt(pow(radius_car - side*(width/2. + del_width), 2) + pow(length + del_length, 2));
  double radius_right_front = sqrt(pow(radius_car + side*(width/2. + del_width), 2) + pow(length + del_length, 2));

  if (pt[0] > -del_length && pt[0] < length+del_length && abs(pt[1]) < width/2 + del_width) {
    // Point inside car
    return 0;
  } else if (side == 0) {
    // Straight
    if (is_straight_collision(pt)) {
      return pt[0] - (length + del_length);
    }
    return MAX_DIST;
  } else {
    // Curve
    // double radius_inner_back  = (side == 1) * radius_left_back   + (side == -1) * radius_right_back;
    double radius_inner_front = (side == 1) * radius_left_front  + (side == -1) * radius_right_front;
    double radius_outer_back  = (side == 1) * radius_right_back  + (side == -1) * radius_left_back;
    double radius_outer_front = (side == 1) * radius_right_front + (side == -1) * radius_left_front;
    double radius_min = radius_car - (width/2. + del_width);

    path.vehicle_radius_min = radius_min;
    path.vehicle_radius_max = radius_outer_front;

    if (is_inner_collision(radius_pt, radius_min, radius_inner_front)) {
      return dist_to_collision_inner(radius_car, side, radius_pt, pt);
    } else if (is_front_collision(radius_pt, radius_inner_front, radius_outer_front)) {
      return dist_to_collision_front(radius_car, side, radius_pt, pt, curvature);
    } else if (is_outer_collision(radius_pt, radius_outer_front, radius_outer_back)) {
      return dist_to_collision_outer(radius_car, side, radius_pt, pt);
    } else {
      return std::min(MAX_ANG * radius_car, MAX_DIST);
    }
  }
}

void Car::calcPathMetrics(Path& path, const std::vector<Vector2f>& point_cloud) {

  float arc_angle_to_point;
  float clearance;
  double free_path_length = std::numeric_limits<float>::max();
  Eigen::Vector2f closest_point = point_cloud[0];
  double min_clearance = 4;
  Eigen::Vector2f clearance_point = closest_point;
  float clearance_fpl;

  // Free path length and closest point
  for (auto& point : point_cloud) {
    double colision_fpl = distance_to_collision(path, point);

    if (colision_fpl < free_path_length) {
      free_path_length = colision_fpl;
      closest_point = point;
    }
  }
  
  // Clearance
  for (auto& point : point_cloud) {
    
    if (free_path_length == 0.) {
      min_clearance = 0;
    } else {
      arc_angle_to_point = fmod(atan2(point[0], -path.side*point[1] + path.radius) + 2*M_PI, 2*M_PI);
      
      // TOOD: Figure out if we need to shrink the free path length
      // new_free_path_length = free_path_lengh - *something*
      
      // TODO: Make Clearance work with a 0 curvature
      if (arc_angle_to_point < abs(path.curvature*free_path_length)) {
        clearance_fpl = arc_angle_to_point * path.radius;

        double radius_pt = (point - Vector2f(0., path.side*path.radius)).norm();
        
        if (radius_pt < path.vehicle_radius_min) {
          clearance = abs(path.vehicle_radius_min - radius_pt);
        } else if (radius_pt > path.vehicle_radius_max) {
          clearance = abs(radius_pt - path.vehicle_radius_max);
        } else {
          // Should never happen, we collided with this point
          clearance = 0;
        }


        clearance = clearance * (FLAGS_clearance_drop_off * clearance_fpl + 1);

        if (clearance < min_clearance) {
          min_clearance = clearance;
          clearance_point = point;
        }
      }
    }
  }
  
  // Add metrics to the path
  path.add_collision_data(free_path_length, closest_point, min_clearance, clearance_point);
}

float Car::timeOptimalController(float vel, float update_period, float free_path_length, Vector2f goal_point) {
  double distance_to_stop_after_accel = update_period * (vel + .5*max_acceleration*update_period) + pow(vel + max_acceleration*update_period, 2)/(2*max_deceleration);

  // Distance till robot needs to stop
  float goal_dist = goal_point.norm();

  if (goal_dist < .5) {
    goal_dist *= (goal_point[0] > 0);
  }
  
  float stop = std::min(goal_dist, free_path_length);

  if (stop > distance_to_stop_after_accel) {
      // Accelerating or cruising
      return std::min(vel + max_acceleration*update_period, max_speed);
  } else {
      // Decelerating or stopped
      return std::max(vel - max_deceleration*update_period, 0.);
  }
}

