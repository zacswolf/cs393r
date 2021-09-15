
#include <vector>
#include "shared/math/math_util.h"
#include "eigen3/Eigen/Dense"

using Eigen::Vector2f;

#ifndef FORWARD_PREDICT_H
#define FORWARD_PREDICT_H

DEFINE_int32(sensing_latency, 0, "Robot sensing latency in periods of 1/20th sec");
DEFINE_int32(actuation_latency, 0, "Robot actuation latency in periods of 1/20th sec");

namespace forward_predict {

void forwardPredict(std::vector<Vector2f> &point_cloud_pred, float &vel_pred, float &rel_angle_pred, Vector2f &rel_loc_pred, Eigen::Matrix<double,10,1> previous_vel_, Eigen::Matrix<double,10,1> previous_curv_) {

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
}

}  // namespace navigation

#endif  // FORWARD_PREDICT_H
