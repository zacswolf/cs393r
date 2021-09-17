
#include <vector>
#include "shared/math/math_util.h"
#include "eigen3/Eigen/Dense"

using Eigen::Vector2f;

#ifndef FORWARD_PREDICT_H
#define FORWARD_PREDICT_H

using namespace navigation;

DEFINE_int32(sensing_latency, 1, "Robot sensing latency in periods of 1/20th sec");
DEFINE_int32(actuation_latency, 2, "Robot actuation latency in periods of 1/20th sec");

namespace forward_predict {

void forwardPredict(std::vector<Vector2f> &point_cloud_pred, float &vel_pred, float &rel_angle_pred, Vector2f &rel_loc_pred, std::array<double, COMMAND_MEMORY_LENGTH> previous_vel_, std::array<double, COMMAND_MEMORY_LENGTH> previous_curv_, Vector2f &goal_point_pred) {

  int total_latency = FLAGS_actuation_latency + FLAGS_sensing_latency;

  for(int i = total_latency-1; i >= 0; i--) {
    // if (vel_pred < previous_vel_[i]) {
    //   // accelerating
    //   vel_pred = std::min(std::min(previous_vel_[i], FLAGS_max_speed), vel_pred + FLAGS_max_acceleration/20);
    // } else if (vel_pred > previous_vel_[i]) {
    //   // decelerating
    //   vel_pred = std::max(std::max(previous_vel_[i], -FLAGS_max_speed), vel_pred - FLAGS_max_deceleration/20);
    // }

    vel_pred = previous_vel_[i];

    double arc_distance_pred = vel_pred/20;

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
      Eigen::Matrix2f R = r1.toRotationMatrix();

      //std::cout << "Rel Loc:  " << del_rel_loc_pred.transpose() << "\n";

      rel_loc_pred = rel_loc_pred + R * del_rel_loc_pred;
      rel_angle_pred = rel_angle_pred + rel_angle_pred;
    }

  }

  Eigen::Rotation2Df r2(-rel_angle_pred);
  Eigen::Matrix2f R = r2.toRotationMatrix();

  // Update point cloud
  for (uint i = 0; i < point_cloud_pred.size(); i++) {
    Eigen::Vector2f pt = point_cloud_pred[i];
    pt -= rel_loc_pred;
    pt = R * pt;

    point_cloud_pred[i] = pt;
  }

  // Update goal point
  goal_point_pred -= rel_loc_pred;
  goal_point_pred = R * goal_point_pred;
}


}  // namespace navigation

#endif  // FORWARD_PREDICT_H
