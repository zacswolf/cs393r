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
DEFINE_double(p1_local_coords, 400., "The goal for P1");

DEFINE_int32(path_sample_algo, 0, "Choose which path sampling algo to use");

DEFINE_double(clearance_drop_off, .1, "The clearance drop off, use 0 for normal clearance");


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
  printf("Start Navigation Init\n");
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
  printf("End Navigation Init\n");
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
  
  // Draw forward predicted goal point
  visualization::DrawCross(goal_point_pred, .5, 0x555555, local_viz_msg_);
  
  // Draw forward predicted robot location
  visualization::DrawCross(rel_loc_pred, .5, 0x031cfc, local_viz_msg_);

  // Sample curvatures
  vector<Path> path_options;

  if (FLAGS_path_sample_algo == 0){
    for (float curvature = 1.; curvature >= 1./64.; curvature *= 0.85) {
      Path path = Path(curvature);
      path_options.push_back(path);
      Path path2 = Path(-curvature);
      path_options.push_back(path2);
    }
  } else {
    for (float curvature = 1.; curvature >= 1./64.; curvature -= (1./32)) {
        Path path = Path(curvature);
        path_options.push_back(path);
        Path path2 = Path(-curvature);
        path_options.push_back(path2);
    }
  }

  // Get Metrics on all curves
  float distance;

  float free_path_length;
  Vector2f closest_point;

  float arc_angle_to_point;
  float clearance_fpl;

  float min_clearance;
  float clearance;
  Vector2f clearance_point;
  
  for (auto& path : path_options) {
    free_path_length = std::numeric_limits<float>::max();
    closest_point = point_cloud_pred[0];
    min_clearance = 4;
    clearance_point = closest_point;

    // Free path length and closest point
    for (auto& point : point_cloud_pred) {
      distance = distance_to_collision(path.curvature, point);
      
      if (distance < free_path_length) {
        free_path_length = distance;
        closest_point = point;
      }
    }

    // Clearance
    for (auto& point : point_cloud_pred) {

      if (free_path_length == 0.) {
        min_clearance = 0;
      } else {
        arc_angle_to_point = fmod(atan2(point[0], -path.side*point[1] + path.radius) + 2*M_PI, 2*M_PI);

        // TODO: Make Clearance work with a 0 curvature
        if (arc_angle_to_point < abs(path.curvature*free_path_length)) {
          clearance_fpl = arc_angle_to_point * path.radius;
          
          // TODO: Change this to use the inner or outer most radius
          // clearance = abs((point - Vector2f(0., path.side*path.radius)).norm() - path.radius) *
          //            (FLAGS_clearance_drop_off * clearance_fpl + 1);

          int side = (0 < path.curvature) - (path.curvature < 0);
          double radius_pt = (point - Vector2f(0., path.side*path.radius)).norm();

          double radius_left_back = sqrt(pow(path.radius - side*(FLAGS_width/2. + FLAGS_del_width), 2) + pow(FLAGS_del_length, 2));
          double radius_right_back = sqrt(pow(path.radius + side*(FLAGS_width/2. + FLAGS_del_width), 2) + pow(FLAGS_del_length, 2));
          double radius_left_front = sqrt(pow(path.radius - side*(FLAGS_width/2. + FLAGS_del_width), 2) + pow(FLAGS_length + FLAGS_del_length, 2));
          double radius_right_front = sqrt(pow(path.radius + side*(FLAGS_width/2. + FLAGS_del_width), 2) + pow(FLAGS_length + FLAGS_del_length, 2));

          double radius_inner_back  = (side == 1) * radius_left_back   + (side == -1) * radius_right_back;
          // double radius_inner_front = (side == 1) * radius_left_front  + (side == -1) * radius_right_front;
          // double radius_outer_back  = (side == 1) * radius_right_back  + (side == -1) * radius_left_back;
          double radius_outer_front = (side == 1) * radius_right_front + (side == -1) * radius_left_front;

          if (radius_pt < radius_inner_back) {
            clearance = abs(radius_inner_back - radius_pt);
          } else if (radius_pt > radius_outer_front) {
            clearance = abs(radius_pt - radius_outer_front);
          } else {
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

  // Select the "best" curvature
  Path best_path = path_options[0];
  float min_loss = best_path.rate_path(goal_point_pred, previous_curv_[0]);
  float loss;

  for (auto& path : path_options) {
    loss = path.rate_path(goal_point_pred, previous_curv_[0]);
    
    if (loss < min_loss) {
      min_loss = loss;
      best_path = path;
    }
  }

  drive_msg_.curvature = best_path.curvature;

  // TOC
  double distance_to_stop_after_accel = .05 * (vel_pred + .5*FLAGS_max_acceleration*.05) + pow(vel_pred + FLAGS_max_acceleration/20, 2)/(2*FLAGS_max_deceleration);
  
  // Distance till robot needs to stop
  // TODO: This only works when the goal point is strictly in front of the car
  float stop = std::min(goal_point_pred.norm() * (goal_point_pred[0] > 0), best_path.free_path_length);
  
  if (stop > distance_to_stop_after_accel) {
    // Not decelerating
    drive_msg_.velocity = std::min(vel_pred + FLAGS_max_acceleration/20., FLAGS_max_speed);
  } else {
    drive_msg_.velocity = std::max(vel_pred - FLAGS_max_deceleration/20., 0.);
  }


  // Shift previous values, for forward predict
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
  //viz_pub_.publish(local_viz_msg_);
  //viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

}  // namespace navigation
