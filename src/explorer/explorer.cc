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
\file    explorer.cc
\brief   Starter code for explorer.
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
#include "explorer.h"
#include "visualization/visualization.h"
#include <limits>
#include "slam.h"
#include "amrl_msgs/Localization2DMsg.h"

#include <unordered_set>
#include "vector_hash.h"


using Eigen::Vector2f;
using Eigen::Vector3f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;


using namespace math_util;
using namespace ros_helpers;

// Create command line arguments
DEFINE_double(p1_local_coords, 400., "The goal for P1");
DEFINE_int32(path_sample_algo, 0, "Choose which path sampling algo to use");

// Forward Predict
DEFINE_int32(sensing_latency, 1, "Robot sensing latency in periods of 1/20th sec");
DEFINE_int32(actuation_latency, 2, "Robot actuation latency in periods of 1/20th sec");
DEFINE_int32(update_rate, 20, "The update rate of the control loop (Hz)");

// Car
DEFINE_double(max_acceleration, 4., "The max acceleration");
DEFINE_double(max_deceleration, 4., "The max deceleration");
DEFINE_double(max_speed, .5, "The max speed");

DEFINE_double(length, .32385, "The wheel base of the robot");
DEFINE_double(width, .2286, "The track width of the robot");
DEFINE_double(del_length, .088075, "The length margin of the robot");
DEFINE_double(del_width, .0254, "The width margin of the robot");
DEFINE_double(safety_margin, .0, "The safety margin for the robot");
DEFINE_int32(run_counter, 40, "Initialization routine");


namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
ros::Publisher slam_loc_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
VisualizationMsg slam_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;

// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace explorer {

Explorer::Explorer(const string& map_file, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    vehicle_(FLAGS_max_acceleration, FLAGS_max_deceleration, FLAGS_max_speed, FLAGS_length, FLAGS_width, FLAGS_del_length, FLAGS_del_width, FLAGS_safety_margin),
    evidence_grid_(),
    global_planner_(evidence_grid_),
    slam_(),
    frontier_(),
    frontier_update_ready_(true),
    run_counter_(0) {

  //global_planner_.evidence_grid_ = &evidence_grid_;

  frontier_point_ = Eigen::Vector2f(0,0);
  printf("Start Explorer Init\n");
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  slam_loc_pub_ = n->advertise<amrl_msgs::Localization2DMsg>("localization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  slam_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "slam");
    
  InitRosHeader("base_link", &drive_msg_.header);

  std::fill(std::begin(previous_vel_), std::end(previous_vel_), 0.);
  std::fill(std::begin(previous_curv_), std::end(previous_curv_), 0.);
  printf("End Explorer Init\n");
}

void Explorer::SetNavGoal(const Vector2f& loc, float angle) {
  nav_goal_loc_ = loc;
  nav_goal_angle_ = angle;

  global_planner_.SetGlobalNavGoal(loc);
  global_planner_.ComputeGlobalPath(robot_loc_, robot_angle_); 
}

void Explorer::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Explorer::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {

  // Slam
  slam_.ObserveOdometry(loc, angle);

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



void Explorer::ObservePointCloud(const vector<Vector2f>& cloud,
                                 const vector<Vector2f>& cloud_open) {
  point_cloud_ = cloud;
  
  // SLAM
  vector<Vector2f> new_points;
  vector<Vector2f> new_open_points;
  bool soft_update = (!frontier_.no_frontier_) && global_planner_.at_path_end_ && !slam_.last_pose_soft_;
  // global_planner_.at_path_end_ = false;

  if (soft_update){
    cout << "<Exp OPC> Soft Update==================\n";
  }

  static double t_glpath_last = 0; // for rate limit
  
  bool slam_update = slam_.ObservePointCloud(cloud, cloud_open, new_points, new_open_points, soft_update);
  if (!frontier_update_ready_ && slam_update) {
    t_glpath_last = GetMonotonicTime();
  }
  frontier_update_ready_ |= slam_update;
  
  Vector2f robot_loc(0,0);
  float robot_angle = 0;
  // works with soft updates
  slam_.GetPoseNoOdom(&robot_loc, &robot_angle);

  std::unordered_set<Vector2i, matrix_hash<Eigen::Vector2i>> new_walls;
  evidence_grid_.UpdateEvidenceGrid(new_points, new_open_points, robot_loc, robot_angle, new_walls, soft_update);
  global_planner_.updateWallGrid(new_walls, soft_update);
  
  static int slam_update_timer = 0;
  if (slam_update) {
    std::cout << "SLAM Updated\n";
    slam_update_timer = GetMonotonicTime();
    // slam_update_counter++;
    // if (slam_update_counter >= 4) {
    //   slam_update_counter = 0;
    //   frontier_point_ = frontier_.findFrontier(evidence_grid_, robot_loc_);
    //   std::cout << "Frontier Loc: " << frontier_point_.transpose() << "\n";
    //   SetNavGoal(frontier_point_, 0);
    // }
  }

  bool timer_update = GetMonotonicTime() - t_glpath_last > 8;
  bool slam_timer_update = GetMonotonicTime() - slam_update_timer > 12;
  if (slam_.isInitialized() && (soft_update || (timer_update && frontier_update_ready_) || slam_timer_update)) {
  // if (slam_update) {
    t_glpath_last = GetMonotonicTime();
    slam_update_timer = GetMonotonicTime();
    //std::cout << "SLAM Updated\n";
    // Go to a new frontier
    // Step 1: Find frontier point
    // Step 2: Set frontier point as new global

    frontier_point_ = frontier_.findFrontier(evidence_grid_, robot_loc_);
    std::cout << "Frontier Loc: " << frontier_point_.transpose() << "\n";
    frontier_update_ready_ = soft_update;
    SetNavGoal(frontier_point_, 0);
  }

  // Vis slam map
  static double t_last = 0.5; // for rate limit
  if (GetMonotonicTime() - t_last > 1) {
    t_last = GetMonotonicTime();
    slam_viz_msg_.header.stamp = ros::Time::now();
    visualization::ClearVisualizationMsg(slam_viz_msg_);

    visualization::DrawCross(frontier_point_, 2., 0x000000, slam_viz_msg_);

    // Plot global map
    evidence_grid_.PlotEvidenceVis(slam_viz_msg_);
    //global_planner_.PlotWallVis(slam_viz_msg_);
    evidence_grid_.PlotNeighborsVis(frontier_point_, slam_viz_msg_);

    // Plot global Path
    global_planner_.PlotGlobalPathVis(slam_viz_msg_);


    viz_pub_.publish(slam_viz_msg_);
  }

  


  // Post localization
  // Vector2f robot_loc(0, 0);
  // float robot_angle(0);
  slam_.GetPose(&robot_loc, &robot_angle);
  amrl_msgs::Localization2DMsg localization_msg;
  localization_msg.pose.x = robot_loc.x();
  localization_msg.pose.y = robot_loc.y();
  localization_msg.pose.theta = robot_angle;
  slam_loc_pub_.publish(localization_msg);
}

void Explorer::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  vehicle_.drawBoundingBox(local_viz_msg_);

  // Draw goal point p1, relative to car
  // const Vector2f goal_point = Vector2f(FLAGS_p1_local_coords - (odom_start_loc_ - odom_loc_).norm(), 0.);
  
  // Eigen::Rotation2Df r_goal(-robot_angle_);
  // // In robot frame
  // const Vector2f goal_point = r_goal * (nav_goal_loc_ - robot_loc_);
  // visualization::DrawCross(goal_point, .5, 0x39B81D, local_viz_msg_);
  Vector2f goal_point = Vector2f(0,0);
  bool is_backward = false;

  if (global_planner_.IsReady()) {
    global_planner_.CheckPathValid(robot_loc_, robot_angle_);

    Vector3f loc_nav_goal = global_planner_.GetLocalNavGoal(robot_loc_, robot_angle_);
    goal_point = loc_nav_goal.segment(0,2);
    is_backward = loc_nav_goal[2];
    

    global_planner_.PlotLocalPathVis(local_viz_msg_);
  }


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

  // Sample paths
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

  // Calculate Path metrics
  for (Path& path : path_options) {
    vehicle_.calcPathMetrics(path, point_cloud_pred);
  }

  // // Visualize paths
  // for (Path& path : path_options) {
  //   path.visualize(local_viz_msg_);
  // }

  // Select the "best" path
  Path best_path = path_options[0];
  float min_loss = best_path.rate_path_alt(goal_point_pred, previous_curv_[0]);
  
  for (auto& path : path_options) {
    float loss = path.rate_path_alt(goal_point_pred, previous_curv_[0]);

    
    if (loss < min_loss) {
      min_loss = loss;
      best_path = path;
    }
  }

  best_path.visualize(local_viz_msg_);

  

  // TOC
  drive_msg_.velocity = vehicle_.timeOptimalController(vel_pred, 1./FLAGS_update_rate, best_path.free_path_length, goal_point_pred, is_backward);

  drive_msg_.curvature = best_path.curvature;
  
  // Negate curvature if velocity is in the wrong direction
  if (drive_msg_.velocity > 0 && is_backward){
    drive_msg_.curvature = -drive_msg_.curvature;
  } else if (drive_msg_.velocity < 0 && !is_backward){
    drive_msg_.curvature = -drive_msg_.curvature;
  }
  

  if (run_counter_ < FLAGS_run_counter && slam_.isInitialized()) {
    std::cout << "<Explorer Run> Init routine!!\n";
    drive_msg_.curvature = 1;
    drive_msg_.velocity = 0.5;
    run_counter_++;
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
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

void Explorer::forwardPredict(std::vector<Vector2f> &point_cloud_pred, float &vel_pred, float &rel_angle_pred, Vector2f &rel_loc_pred, std::array<double, COMMAND_MEMORY_LENGTH> previous_vel_, std::array<double, COMMAND_MEMORY_LENGTH> previous_curv_, Vector2f &goal_point_pred) {

  int total_latency = FLAGS_actuation_latency + FLAGS_sensing_latency;

  for(int i = total_latency-1; i >= 0; i--) {
    vel_pred = previous_vel_[i];
    double arc_distance_pred = vel_pred/FLAGS_update_rate;

    if (previous_curv_[i] == 0.) {
      // Driving straight
      rel_loc_pred += Vector2f(cos(rel_angle_pred), sin(rel_angle_pred)) * arc_distance_pred;
    } else {
      // Turning
      double del_rel_angle_pred = arc_distance_pred * previous_curv_[i];
      double radius = 1/abs(previous_curv_[i]);
      int side = (0 < previous_curv_[i]) - (previous_curv_[i] < 0);
      Eigen::Vector2f del_rel_loc_pred = Vector2f(sin(abs(del_rel_angle_pred)), 
                                                  side * (1 - cos(abs(del_rel_angle_pred)))) * radius;

      // Rotation matrix
      Eigen::Rotation2Df r1(rel_angle_pred);
      rel_loc_pred = rel_loc_pred + r1 * del_rel_loc_pred;
      rel_angle_pred = rel_angle_pred + rel_angle_pred;
    }

  }

  Eigen::Rotation2Df r2(-rel_angle_pred);

  // Update point cloud
  for (uint i = 0; i < point_cloud_pred.size(); i++) {
    Eigen::Vector2f pt = point_cloud_pred[i];
    pt -= rel_loc_pred;
    pt = r2 * pt;

    point_cloud_pred[i] = pt;
  }

  // Update goal point
  goal_point_pred -= rel_loc_pred;
  goal_point_pred = r2 * goal_point_pred;
}

}  // namespace explorer
