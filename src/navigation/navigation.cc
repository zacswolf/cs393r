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
#include "collision.h"
#include "path.h"
#include "forward_predict.h"

#include <limits>

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;
using namespace collision;
using namespace forward_predict;

// Create command line arguments
DEFINE_double(p1_local_coords, 6., "The goal for P1");

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

  std::fill(std::begin(previous_vel_), std::end(previous_vel_), 0.);
  std::fill(std::begin(previous_curv_), std::end(previous_curv_), 0.);
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

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // Draw goal point p1, relative to car
  const Vector2f goal_point = Vector2f(FLAGS_p1_local_coords - (odom_start_loc_ - odom_loc_).norm(), 0.);
  // const Vector2f goal_point2 = Vector2f(FLAGS_p1_local_coords, 0.) + odom_start_loc_ - odom_loc_;
  visualization::DrawCross(goal_point, .5, 0x39B81D, local_viz_msg_);

  // Draw car bounding box
  Vector2f p_bl = Vector2f(-FLAGS_del_length, FLAGS_width/2 + FLAGS_del_width);
  Vector2f p_br = Vector2f(-FLAGS_del_length, -1*(FLAGS_width/2 + FLAGS_del_width));
  Vector2f p_fl = Vector2f(FLAGS_length + FLAGS_del_length, FLAGS_width/2 + FLAGS_del_width);
  Vector2f p_fr = Vector2f(FLAGS_length + FLAGS_del_length, -1*(FLAGS_width/2 + FLAGS_del_width));
  visualization::DrawLine(p_bl, p_fl, 0x00FFAA, local_viz_msg_);
  visualization::DrawLine(p_br, p_fr, 0x00FFAA, local_viz_msg_);
  visualization::DrawLine(p_fl, p_fr, 0x00FFAA, local_viz_msg_);
  visualization::DrawLine(p_bl, p_br, 0x00FFAA, local_viz_msg_);

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.

  // Forward-predict
  float odom_vel = robot_vel_.norm();
  std::vector<Eigen::Vector2f> point_cloud_pred = point_cloud_;
  Eigen::Vector2f rel_loc_pred = Vector2f(0., 0.);
  Eigen::Vector2f goal_point_pred = goal_point;
  float rel_angle_pred = 0.;
  float vel_pred = odom_vel;
  forwardPredict(point_cloud_pred, vel_pred, rel_angle_pred, rel_loc_pred, previous_vel_, previous_curv_, goal_point_pred);
  
  visualization::DrawCross(goal_point_pred, .5, 0x555555, local_viz_msg_);

  // printf("rel_loc_pred %f %f\n", rel_loc_pred[0], rel_loc_pred[1]);
  visualization::DrawCross(rel_loc_pred, .5, 0x031cfc, local_viz_msg_);

  // Sample n curvatures
  vector<Path> path_options;

  for (float curvature = -1.; curvature <= -1./64.; curvature *= 0.5) {
    Path path = Path(curvature);
    path.curvature = curvature;
    path_options.push_back(path);
  }

  for (float curvature = 1.; curvature >= 1./64.; curvature *= 0.5) {
    Path path = Path(curvature);
    path.curvature = curvature;
    path_options.push_back(path);
  }

  // Get Metrics on all curves
  float distance;
  float clearance;
  float free_path_length;
  float min_clearance;
  Vector2f closest_point;
  Vector2f clearance_point;
  float arc_angle_to_point;
  
  for (auto& path : path_options) {
    free_path_length = std::numeric_limits<float>::max();
    closest_point = point_cloud_pred[0];
    min_clearance = 4;
    clearance_point = closest_point;

    for (auto& point : point_cloud_pred) {
      distance = distance_to_collision(path.curvature, point);
      
      if (distance < free_path_length) {
        free_path_length = distance;
        closest_point = point;
      }
    }

    for (auto& point : point_cloud_pred) {
      arc_angle_to_point = fmod(atan2(point[0], -path.side*point[1] + path.radius) + 2*M_PI, 2*M_PI);

      if (arc_angle_to_point < abs(path.curvature*free_path_length)) {
        clearance = abs((point - Vector2f(0., path.side*path.radius)).norm() - path.radius);
        if (clearance < min_clearance) {
          min_clearance = clearance;
          clearance_point = point;
        }
      }
    }

    path.add_collision_data(free_path_length, closest_point, min_clearance, clearance_point);
  }

  // Visualize arcs
  int side;
  float start_angle;
  float end_angle;
  Eigen::Vector2f center;

  for (auto& path : path_options) {
    if (path.curvature == 0) {
      visualization::DrawLine(Vector2f(0., 0.), Vector2f(path.free_path_length, 0.), 0xfca903, local_viz_msg_);
    } else {
      side = (0 < path.curvature) - (path.curvature < 0);
      start_angle = side * -M_PI/2;
      end_angle = start_angle + path.free_path_length * path.curvature;
      center = Vector2f(0, 1/path.curvature);
      
      if (side == 1) {
        start_angle = -side*M_PI/2;
        end_angle = start_angle + path.free_path_length * path.curvature;
      } else {
        start_angle = start_angle + path.free_path_length * path.curvature;
        end_angle = -side*M_PI/2;
      }

      visualization::DrawArc(center,
              path.radius,
              start_angle,
              end_angle,
              0xfca903,
              local_viz_msg_);
    }
  }

  // Experimental
  Eigen::Vector2f closest_barrier_point = point_cloud_pred[0];
  float dx;
  float dy;
  float side_to_point_distance;

  float min_distance = std::numeric_limits<float>::max();
  for (auto& pt : point_cloud_pred) {
    dx = std::max(std::max(-FLAGS_del_length - pt[0], 0.), pt[0] - (FLAGS_length + FLAGS_del_length));
    dy = std::max(std::max(-(FLAGS_width/2 + FLAGS_del_width) - pt[1], 0.), pt[1] - (FLAGS_width/2 + FLAGS_del_width));
    
    // Relative to side of the car
    side_to_point_distance = sqrt(dx*dx + dy*dy);

    if (side_to_point_distance < min_distance) {
      closest_barrier_point = pt;
      min_distance = side_to_point_distance;
    }
  }

  // Select the "best" curvature
  Path best_path = path_options[0];
  float min_loss = best_path.rate_path1(goal_point_pred, closest_barrier_point);
  float loss;

  for (auto& path : path_options) {
    loss = path.rate_path1(goal_point_pred, closest_barrier_point);
    //printf("Path %f %f %f %f \n", path.curvature, path.free_path_length, path.closest_point[1], loss);
    
    if (loss < min_loss) {
      min_loss = loss;
      best_path = path;
    }
  }

  drive_msg_.curvature = best_path.curvature;

  // TOC

  // Distance it takes the robot to stop if it's driving full speed
  // double max_distance_to_stop = pow(FLAGS_max_speed, 2)/(2*FLAGS_max_deceleration);

  // Distance it takes the robot to stop if it's driving at the current speed
  // double distance_to_stop = pow(vel_pred, 2)/(2*FLAGS_max_deceleration);

  double distance_to_stop_after_accel = .05 * (vel_pred + .5*FLAGS_max_acceleration*.05) + pow(vel_pred + FLAGS_max_acceleration/20, 2)/(2*FLAGS_max_deceleration);
  // (vel_pred + (vel_pred + FLAGS_max_acceleration/20.)) / (2. * 20.) + pow(vel_pred + FLAGS_max_acceleration/20, 2)/(2*FLAGS_max_deceleration);
  //std::max(odom_vel, vel_pred)
  

  // Distance till robot needs to stop
  // TODO: This only works when the goal point is strictly in front of the car
  float stop = std::min(goal_point_pred.norm() * (goal_point_pred[0] > 0), best_path.free_path_length);
  
  printf("%i %f %f\n", vel_pred < FLAGS_max_speed, stop, distance_to_stop_after_accel);
  if (vel_pred < FLAGS_max_speed && stop > distance_to_stop_after_accel) {
    // accelerating
    printf("accelerating\n");
    drive_msg_.velocity = std::min(FLAGS_max_acceleration/20 + vel_pred, FLAGS_max_speed);
  } else {
    // Robot needs to stop
    printf("stop\n");
    if (odom_vel != 0. && vel_pred != 0.){
      printf("%f %f\n", odom_vel, vel_pred);
    }
    drive_msg_.velocity = std::max(vel_pred - FLAGS_max_deceleration/20., 0.);
  }

  // shift previous values
  for(int i = COMMAND_MEMORY_LENGTH-2; i >= 0; i--){
    previous_vel_[i+1] = previous_vel_[i];
    previous_curv_[i+1] = previous_curv_[i];
  }
  previous_vel_[0] = drive_msg_.velocity;
  previous_curv_[0] = drive_msg_.curvature;

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
