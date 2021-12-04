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
\file    explorer_main.cc
\brief   Main entry point for reference Explorer implementation
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>

#include "glog/logging.h"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/Localization2DMsg.h"
#include "gflags/gflags.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"

#include "explorer.h"

using amrl_msgs::Localization2DMsg;
using math_util::DegToRad;
using math_util::RadToDeg;
using explorer::Explorer;
using ros::Time;
using ros_helpers::Eigen3DToRosPoint;
using ros_helpers::Eigen2DToRosPoint;
using ros_helpers::RosPoint;
using ros_helpers::SetRosVector;
using std::string;
using std::vector;
using Eigen::Vector2f;

// Create command line arguments
DEFINE_string(laser_topic, "scan", "Name of ROS topic for LIDAR data");
DEFINE_string(odom_topic, "odom", "Name of ROS topic for odometry data");
DEFINE_string(loc_topic, "localization", "Name of ROS topic for localization");
DEFINE_string(init_topic,
              "initialpose",
              "Name of ROS topic for initialization");
DEFINE_string(map, "maps/GDC1.txt", "Name of vector map file");

bool run_ = true;
sensor_msgs::LaserScan last_laser_msg_;
Explorer* explorer_ = nullptr;

// From nav
void LocalizationCallback(const amrl_msgs::Localization2DMsg msg) {
  if (FLAGS_v > 0) {
    printf("Localization t=%f\n", GetWallTime());
  }
  explorer_->UpdateLocation(Vector2f(msg.pose.x, msg.pose.y), msg.pose.theta);
}

void LaserCallback(const sensor_msgs::LaserScan& msg) {
  if (FLAGS_v > 0) {
    printf("Laser t=%f, dt=%f\n",
           msg.header.stamp.toSec(),
           GetWallTime() - msg.header.stamp.toSec());
  }
  // Location of the laser on the robot. Assumes the laser is forward-facing.
  const Vector2f kLaserLoc(0.2, 0);

  // Find number of rays from the LaserScan
  const int num_rays = static_cast<int>(
      1.0 + (msg.angle_max - msg.angle_min) /
      msg.angle_increment);

  static vector<Vector2f> point_cloud_(num_rays);
  point_cloud_.clear();

  static vector<Vector2f> point_cloud_open_(num_rays);
  point_cloud_open_.clear();

  // Convert the LaserScan to a point cloud
  for (int i = 0; i < num_rays; i++) {
    float range = msg.ranges[i];
    float angle = msg.angle_min + i * msg.angle_increment;
    Vector2f laser_loc = Vector2f(range * cos(angle), range * sin(angle)) + kLaserLoc;
    if (range > msg.range_min && range < msg.range_max) {
      point_cloud_.push_back(laser_loc);
    } else {
      point_cloud_open_.push_back(laser_loc);
    }
  }

  explorer_->ObservePointCloud(point_cloud_, point_cloud_open_);

  // PublishMap();
  // PublishPose();
}

void OdometryCallback(const nav_msgs::Odometry& msg) {
  if (FLAGS_v > 0) {
    printf("Odometry t=%f\n", msg.header.stamp.toSec());
  }
  explorer_->UpdateOdometry(
      Vector2f(msg.pose.pose.position.x, msg.pose.pose.position.y),
      2.0 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
      Vector2f(msg.twist.twist.linear.x, msg.twist.twist.linear.y),
      msg.twist.twist.angular.z);
}

void GoToCallback(const geometry_msgs::PoseStamped& msg) {
  const Vector2f loc(msg.pose.position.x, msg.pose.position.y);
  const float angle =
      2.0 * atan2(msg.pose.orientation.z, msg.pose.orientation.w);
  printf("Goal: (%f,%f) %f\u00b0\n", loc.x(), loc.y(), angle);
  explorer_->SetNavGoal(loc, angle);
}

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);
  // Initialize ROS.
  ros::init(argc, argv, "navigation", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  explorer_ = new Explorer(FLAGS_map, &n);

  // visualization_publisher_ =
  //     n.advertise<VisualizationMsg>("visualization", 1);
  // localization_publisher_ =
  //     n.advertise<amrl_msgs::Localization2DMsg>("localization", 1);

  ros::Subscriber velocity_sub =
      n.subscribe(FLAGS_odom_topic, 1, &OdometryCallback);
  ros::Subscriber localization_sub =
      n.subscribe(FLAGS_loc_topic, 1, &LocalizationCallback);
  ros::Subscriber laser_sub =
      n.subscribe(FLAGS_laser_topic, 1, &LaserCallback);
  ros::Subscriber goto_sub =
      n.subscribe("/move_base_simple/goal", 1, &GoToCallback);

  printf("Nav main start run loop\n");
  RateLoop loop(20.0);
  while (run_ && ros::ok()) {
    ros::spinOnce();
    explorer_->Run();
    loop.Sleep();
  }
  delete explorer_;
  return 0;
}
