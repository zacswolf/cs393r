
#include <vector>

#include "eigen3/Eigen/Dense"
#include "visualization/visualization.h"
#include "amrl_msgs/VisualizationMsg.h"

using Eigen::Vector2f;

#ifndef PATH_H
#define PATH_H

class Path {
 public:
  float curvature;
  float radius;
  int side;

  float vehicle_radius_min;
  float vehicle_radius_max;

  float clearance;
  float free_path_length;

  float arc_angle;
  Eigen::Vector2f car_pos;

  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  Eigen::Vector2f clearance_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  explicit Path(float curvature);

  float point_to_path_dist(const Vector2f& goal_point);

  void add_collision_data(float free_path_length, Vector2f closest_point, float clearance, Vector2f clearance_point);

  float rate_path(const Vector2f& goal_point, float previous_curv);

  float rate_path_andrew(const Vector2f& goal_point, float previous_curv);

  void visualize(amrl_msgs::VisualizationMsg& local_viz_msg_);
};

#endif  // PATH_H
