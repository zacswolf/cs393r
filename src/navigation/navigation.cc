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
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

// Create command line arguments
DEFINE_double(p1_local_coords, 10., "The goal for P1");
DEFINE_double(max_acceleration, 4., "The max acceleration");
DEFINE_double(max_deceleration, 4., "The max deceleration");
DEFINE_double(max_speed, 1., "The max speed");

DEFINE_double(length, .5, "The wheel base of the robot");
DEFINE_double(width, .2, "The track width of the robot");
DEFINE_double(del_length, .1, "The length margin of the robot");
DEFINE_double(del_width, .1, "The width margin of the robot");
// DEFINE_double(safety_margin, .15, "The safety margin for the robot");

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                                     
}


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
  return (abs(pt[0]) <= FLAGS_width/2 + FLAGS_del_width);
}

// Distance to collision
double dist_to_collision_inner(double radius_car, int side, double radius_pt, Vector2f pt) {
  double theta_car = acos(-side * (FLAGS_width/2 + FLAGS_del_width) / radius_pt);
  double theta_point = atan2(pt[0], radius_car - side*pt[1]);
  return (theta_point - theta_car) * radius_car;
}

double dist_to_collision_front(double radius_car, int side, double radius_pt, Vector2f pt) {
  double theta_car = asin((FLAGS_length + FLAGS_del_length) / radius_pt);
  double theta_point = atan2(pt[0], radius_car - side*pt[1]);
  return (theta_point - theta_car) * radius_car;
}

double dist_to_collision_outer(double radius_car, int side, double radius_pt, Vector2f pt) {
  double theta_car = acos(side * (FLAGS_width/2 + FLAGS_del_width) / radius_pt);
  double theta_point = atan2(pt[0], radius_car + side*pt[1]);
  return (theta_point - theta_car) * radius_car;
}


// big boi
double distance_to_collision(double curvature, Vector2f pt) {

  double radius_car = 1/abs(curvature);
  int side = (2*(curvature>0) - 1) * (curvature != 0);
  double radius_pt = (pt-Vector2f(0.,radius_car)).norm();

  double radius_left_back = sqrt(pow(radius_car - side*(FLAGS_width/2. + FLAGS_del_width), 2) + pow(FLAGS_del_length, 2));
  double radius_right_back = sqrt(pow(radius_car + side*(FLAGS_width/2. + FLAGS_del_width), 2) + pow(FLAGS_del_length, 2));
  double radius_left_front = sqrt(pow(radius_car - side*(FLAGS_width/2. + FLAGS_del_width), 2) + pow(FLAGS_length + FLAGS_del_length, 2));
  double radius_right_front = sqrt(pow(radius_car + side*(FLAGS_width/2. + FLAGS_del_width), 2) + pow(FLAGS_length + FLAGS_del_length, 2));


  if (side == 0) {
    // Straight
    if (is_straight_collision(pt)) {
      return pt[0] - (FLAGS_length + FLAGS_del_length);
    }
    return -1;
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

    printf("Radii: %.3f, %.3f, %.3f\n", radius_pt, radius_inner_front, radius_outer_front);

    if (is_inner_collision(radius_pt, radius_inner_back, radius_inner_front)) {
      printf("Inner collision\n");
      return dist_to_collision_inner(radius_car, side, radius_pt, pt);
    } else if (is_front_collision(radius_pt, radius_inner_front, radius_outer_front)) {
      printf("Front collision\n");
      return dist_to_collision_front(radius_car, side, radius_pt, pt);
    } else if (is_outer_collision(radius_pt, radius_outer_front, radius_outer_back)) {
      return dist_to_collision_outer(radius_car, side, radius_pt, pt);
    } else {
      return -1;
    }
  }
}




void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // Draw goal point p1
  const Vector2f goal_point = Vector2f(FLAGS_p1_local_coords-(odom_start_loc_-odom_loc_).norm(), 0);
  visualization::DrawCross(goal_point, .5, 0x39B81D, local_viz_msg_);

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"

  static Vector2f zero = Vector2f(0., 0.);

  for (uint i = 0; i < point_cloud_.size(); i += 100) {
    auto point = point_cloud_[i];
    visualization::DrawLine(zero, point, 0xDE0000, local_viz_msg_);
  }

  // TODO: Sample n curvatures

  // TODO: Select the "best" curvature
  double curvature = .1;
  std::cout << distance_to_collision(curvature, goal_point) << "\n";

  drive_msg_.curvature = curvature;

  // TOC for selected_curvature
  double remaining_distance = goal_point.x() + goal_point.y();

  if (pow(robot_vel_.norm(), 2)/(2*FLAGS_max_deceleration) > remaining_distance) {
    // We need to stop
    drive_msg_.velocity = 0;
  } else {
    // Go max speed
    drive_msg_.velocity = FLAGS_max_speed;
  }

  // drive_msg_.curvature = 0.;
  // drive_msg_.velocity = 1.;

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

}  // namespace navigation
