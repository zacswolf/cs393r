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

#include <limits>

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;
using namespace collision;

// Create command line arguments
DEFINE_double(p1_local_coords, 50., "The goal for P1");
DEFINE_int32(sensing_latency, 0, "Robot sensing latency in periods of 1/20th sec");
DEFINE_int32(actuation_latency, 0, "Robot actuation latency in periods of 1/20th sec");

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

  previous_vel_.setZero();
  previous_curv_.setZero();
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
  const Vector2f goal_point = Vector2f(FLAGS_p1_local_coords - (odom_start_loc_ - odom_loc_).norm(), 0);
  // const Vector2f goal_point2 = Vector2f(FLAGS_p1_local_coords, 0) + odom_start_loc_ - odom_loc_;
  visualization::DrawCross(goal_point, .5, 0x39B81D, local_viz_msg_);

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"

  /*
  static Vector2f zero = Vector2f(0., 0.);

  for (uint i = 0; i < point_cloud_.size(); i += 100) {
    auto point = point_cloud_[i];
    visualization::DrawLine(zero, point, 0xDE0000, local_viz_msg_);
  }
  */
  // Forward-predict
  
  double odom_vel = pow(pow(robot_vel_(0),2) + pow(robot_vel_(1),2),0.5);

  std::vector<Eigen::Vector2f> point_cloud_pred = point_cloud_;
  Eigen::Vector2f rel_loc_pred;
  rel_loc_pred(0) = 0;
  rel_loc_pred(1) = 0;
  double rel_angle_pred = 0;
  double vel_pred = odom_vel;

  int total_latency = FLAGS_actuation_latency + FLAGS_sensing_latency;
  int actuation_latency = FLAGS_actuation_latency;

  for(int i=total_latency-1;i>=actuation_latency-1;i--){

    if (previous_vel_(i) > vel_pred) {
      // accelerating
      vel_pred = std::min(std::min(previous_vel_(i),FLAGS_max_speed), vel_pred + FLAGS_max_deceleration/20);
    } else {
      // decelerating
      vel_pred = std::max(std::max(previous_vel_(i),-FLAGS_max_speed), vel_pred - FLAGS_max_deceleration/20);
    }

    double arc_distance_pred = vel_pred/20;

    if (previous_curv_(i) == 0) {
      // Driving straight
      rel_loc_pred(0) = rel_loc_pred(0) + cos(rel_angle_pred)*arc_distance_pred;
      rel_loc_pred(1) = rel_loc_pred(1) + sin(rel_angle_pred)*arc_distance_pred;
    } else {
      // Turning
      double del_rel_angle_pred = arc_distance_pred*previous_curv_(i);
      double radius = 1/abs(previous_curv_(i));
      int side = (2*(previous_curv_(i)>0) - 1) * (previous_curv_(i) != 0);
      Eigen::Vector2f del_rel_loc_pred;
      del_rel_loc_pred(0) = radius*sin(abs(del_rel_angle_pred));
      del_rel_loc_pred(1) = side*(radius - radius*cos(abs(del_rel_angle_pred)));

      // Rotation matrix
      Eigen::Matrix2f R;
      R(0,0) = cos(rel_angle_pred);
      R(0,1) = -sin(rel_angle_pred);
      R(1,0) = sin(rel_angle_pred);
      R(1,1) = cos(rel_angle_pred);

      //std::cout << "Rel Loc:  " << del_rel_loc_pred.transpose() << "\n";

      rel_loc_pred = rel_loc_pred + R*del_rel_loc_pred;
      rel_angle_pred = rel_angle_pred + rel_angle_pred;
    }

  }

  Eigen::Matrix2f R;
  R(0,0) = cos(-rel_angle_pred);
  R(0,1) = -sin(-rel_angle_pred);
  R(1,0) = sin(-rel_angle_pred);
  R(1,1) = cos(-rel_angle_pred);

  // Update point cloud
  for (int i=0; i<((int) point_cloud_pred .size()); i++) {
    Eigen::Vector2f pt = point_cloud_pred[i];
    pt = pt - rel_loc_pred;
    pt = R*pt;

    point_cloud_pred[i] = pt;
  }

  //visualization::DrawCross(rel_loc_pred, .5, 0x031cfc, local_viz_msg_);

  //std::cout << "\n\n";

  // TODO: Sample n curvatures
  vector<Path> path_options;

  for (float curvature = -2.; curvature <= -1./64.; curvature *= 0.5) {
    Path path = Path(curvature);
    path.curvature = curvature;
    path_options.push_back(path);
  }

  for (float curvature = 2.; curvature >= 1./64.; curvature *= 0.5) {
    Path path = Path(curvature);
    path.curvature = curvature;
    path_options.push_back(path);
  }

  // Eigen::Matrix<float,16,1> radius_list;
  // radius_list << -50,-5,-4,-3,-2,-1.75,-1.5,-1.25,-1,-0.75,-0.5,0.5,0.75,1,0.8,1,2,3,4,5,50;

  // for (int i=0; i<16; i++) {
  //   float curvature = 1/radius_list[i];
  //   Path path = Path(curvature);
  //   path.curvature = curvature;
  //   path_options.push_back(path);
  // }

  // for (float radius = -20; radius <= -0.5; radius += 0.25) {
  //   if (radius != 0){
  //     float curvature = 1/radius;
  //     Path path = Path(curvature);
  //     path.curvature = curvature;
  //     path_options.push_back(path);
  //   }
  // }

  // for (float radius = 0.5; radius <= 20; radius += 0.25) {
  //   if (radius != 0){
  //     float curvature = 1/radius;
  //     Path path = Path(curvature);
  //     path.curvature = curvature;
  //     path_options.push_back(path);
  //   }
  // }
  // Path path1 = Path(1/20);
  // path1.curvature = 1/20;
  // path_options.push_back(path1);

  // Path path2 = Path(-1/20);
  // path2.curvature = -1/20;
  // path_options.push_back(path2);
//   Eigen::Matrix<double,13,1> curv_set_;
//   Eigen::Matrix<double,13,1> cost_;
//   curv_set_ << -2,-1,-0.5,-0.2,-0.1,-0.05,0.,0.05,0.1,0.2,0.5,1,2;
//   int min_cost_ind = 0;
//   for (int i=0; i<13; i++) {
//     double curv_option = curv_set_(i);
//     double min_dist = 16;

//     for (int j=0; j<((int) point_cloud_.size()); j++){
//       Eigen::Vector2f pt = point_cloud_[j];
//       double dist = distance_to_collision(curv_option, pt);
//       min_dist = std::min(dist,min_dist);
//     }
//     //std::cout << " " << min_dist << "";
//     if (curv_option != 0.) {
//       int side = (2*(curv_option>0) - 1) * (curv_option != 0);
//       Eigen::Vector2f center;
//       center(0) = 0;
//       center(1) = 1/curv_option;
//       float radius = 1/abs(curv_option);
//       float start_angle;
//       float end_angle;
//       if (side == 1) {
//         start_angle = -3.14/2;
//         end_angle = start_angle+min_dist*abs(curv_option);
//       } else {
//         end_angle = 3.14/2;
//         start_angle = end_angle-min_dist*abs(curv_option);
//       }
//       visualization::DrawArc(center,
//               radius,
//               start_angle,
//               end_angle,
//               0xfca903,
//               local_viz_msg_);
//     }
    

//     cost_(i) = 1/min_dist + 1/(abs(curv_option)+1);
//     if (cost_(i) < cost_(min_cost_ind)){
//       min_cost_ind = i;
//     }
//   }

  //double curvature = curv_set_(min_cost_ind);
  //std::cout << "\n\n";
  //std::cout << distance_to_collision(curvature, goal_point) << "\n";

  // Get Metrics on all curves
  float distance;
  float free_path_length;
  Vector2f closest_point;
  for (auto& path : path_options) {
    free_path_length = std::numeric_limits<float>::max();
    closest_point = point_cloud_pred[0];
    for (auto& point : point_cloud_pred) {
      distance = distance_to_collision(path.curvature, point);
      if (distance < free_path_length) {
        free_path_length = distance;
        closest_point = point;
      }
    }
    path.add_collision_data(free_path_length, closest_point);
  }

  // Visualize arcs
  for (auto& path : path_options) {
    // path.curvature
    // path.free_path_length
    if (path.curvature == 0) {
      visualization::DrawLine(Vector2f(0, 0), Vector2f(path.free_path_length, 0), 0xfca903, local_viz_msg_);
    } else {
      int side = (2 * (path.curvature > 0) - 1) * (path.curvature != 0);
      Eigen::Vector2f center = Vector2f(0, 1/path.curvature);
      float start_angle;
      float end_angle;
      if (side == 1) {
        start_angle = -M_PI/2;
        end_angle = start_angle + path.free_path_length * abs(path.curvature);
      } else {
        end_angle = M_PI/2;
        start_angle = end_angle - path.free_path_length * abs(path.curvature);
      }
      visualization::DrawArc(center,
              path.radius,
              start_angle,
              end_angle,
              0xfca903,
              local_viz_msg_);
    }
  }

  Eigen::Vector2f closest_barrier_point;
  closest_barrier_point = point_cloud_pred[0];
  float min_distance = std::numeric_limits<float>::max();
  for (auto& pt : point_cloud_pred) {
    float distance;
    float dx = std::max(std::max(-FLAGS_del_length - pt[0], 0.), pt[0] - (FLAGS_length+FLAGS_del_length));
    float dy = std::max(std::max(-(FLAGS_width/2 + FLAGS_del_width) - pt[1], 0.), pt[1] - (FLAGS_width/2 + FLAGS_del_width));
    distance = sqrt(dx*dx + dy*dy);

    if (distance < min_distance) {
      closest_barrier_point = pt;
      min_distance = distance;
    }
  }

  // TODO: Select the "best" curvature
  Path best_path = path_options[0];
  float min_loss = best_path.rate_path1(goal_point,closest_barrier_point);
  float loss;
  for (auto& path : path_options) {
    loss = path.rate_path1(goal_point,closest_barrier_point);
    //printf("Path %f %f %f %f \n", path.curvature, path.free_path_length, path.closest_point[1], loss);
    if (loss < min_loss) {
      min_loss = loss;
      best_path = path;
    }
  }

  drive_msg_.curvature = best_path.curvature;

  // TOC for selected_curvature
  double remaining_distance = goal_point.x() + goal_point.y();

  if (pow(vel_pred, 2)/(2*FLAGS_max_deceleration) > remaining_distance) {
    // We need to stop
    drive_msg_.velocity = 0;
  } else {
    // Go max speed
    drive_msg_.velocity = FLAGS_max_speed;
  }

  // shift previous values
  for(int i=8;i>=0;i--){
    previous_vel_(i+1) = previous_vel_(i);
    previous_curv_(i+1) = previous_curv_(i);
  }
  previous_vel_(0) = drive_msg_.velocity;
  previous_curv_(0) = drive_msg_.curvature;

  // drive_msg_.curvature = 0.;
  // drive_msg_.velocity = 1.;

  // Draw car bounding box

  Vector2f p_bl = Vector2f(-FLAGS_del_length,FLAGS_width/2+FLAGS_del_width);
  Vector2f p_br = Vector2f(-FLAGS_del_length,-FLAGS_width/2-FLAGS_del_width);
  Vector2f p_fl = Vector2f(FLAGS_length+FLAGS_del_length,FLAGS_width/2+FLAGS_del_width);
  Vector2f p_fr = Vector2f(FLAGS_length+FLAGS_del_length,-FLAGS_width/2-FLAGS_del_width);
  visualization::DrawLine(p_bl,p_fl,0x00FFAA,local_viz_msg_);
  visualization::DrawLine(p_br,p_fr,0x00FFAA,local_viz_msg_);
  visualization::DrawLine(p_fl,p_fr,0x00FFAA,local_viz_msg_);
  visualization::DrawLine(p_bl,p_br,0x00FFAA,local_viz_msg_);

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
