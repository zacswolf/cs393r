
#include <vector>
#include "eigen3/Eigen/Dense"
#include "path.h"
#include "visualization/visualization.h"

#ifndef CAR_H
#define CAR_H

class Car {
public:
    // Constructor:
    explicit Car(double max_acceleration,
        double max_deceleration,
        double max_speed,
        double length,
        double width,
        double del_length,
        double del_width,
        double safety_margin);

    void drawBoundingBox(amrl_msgs::VisualizationMsg& local_viz_msg_);

    void calcPathMetrics(Path& path, const std::vector<Vector2f>& point_cloud);

    float timeOptimalController(float vel, float update_period, float free_path_length, Vector2f goal_point, bool is_backwards);

private:
    double max_acceleration;
    double max_deceleration;
    double max_speed;
    double length;
    double width;
    double del_length;
    double del_width;
    // calculateClearance(Path path)


    // getRadiusInner


    // Existance
    bool is_inner_collision(double radius_pt, double radius_inner_back, double radius_inner_front);
    bool is_front_collision(double radius_pt, double radius_inner_front, double radius_outer_front);
    bool is_outer_collision(double radius_pt, double radius_outer_front, double radius_outer_back);
    bool is_straight_collision(Vector2f pt);

    // Distance to collision
    double dist_to_collision_inner(double radius_car, int side, double radius_pt, Vector2f pt);
    double dist_to_collision_front(double radius_car, int side, double radius_pt, Vector2f pt, double curvature);
    double dist_to_collision_outer(double radius_car, int side, double radius_pt, Vector2f pt);

    // big boi, returns max if no collison 
    double distance_to_collision(Path& path, Vector2f pt);
};

#endif  // CAR_H
