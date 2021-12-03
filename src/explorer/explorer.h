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
\file    explorer.h
\brief   Interface for reference Explorer class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"
#include "car.h"
#include "path.h"
#include "global_planner.h"


#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace explorer {

const int COMMAND_MEMORY_LENGTH = 20;

class Explorer {
 public:

   // Constructor
  explicit Explorer(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

 private:
  void forwardPredict(std::vector<Eigen::Vector2f> &point_cloud_pred,
                      float &vel_pred, float &rel_angle_pred,
                      Eigen::Vector2f &rel_loc_pred,
                      std::array<double, COMMAND_MEMORY_LENGTH> previous_vel_,
                      std::array<double, COMMAND_MEMORY_LENGTH> previous_curv_,
                      Eigen::Vector2f &goal_point_pred);


  // Whether odometry has been initialized.
  bool odom_initialized_;
  // Whether localization has been initialized.
  bool localization_initialized_;
  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;
  // Odometry-reported robot starting location.
  Eigen::Vector2f odom_start_loc_;
  // Odometry-reported robot starting angle.
  float odom_start_angle_;
  // Latest observed point cloud.
  std::vector<Eigen::Vector2f> point_cloud_;

  // Whether explorer is complete.
  bool nav_complete_;
  // Explorer goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Explorer goal angle.
  float nav_goal_angle_;

  // Previous velocity commands
  std::array<double, COMMAND_MEMORY_LENGTH> previous_vel_;

  // Previous curvature commands
  std::array<double, COMMAND_MEMORY_LENGTH> previous_curv_;

  // Vehicle object
  Car vehicle_;

  // Global planner
  Global_Planner global_planner_;

};

}  // namespace explorer

#endif  // NAVIGATION_H
