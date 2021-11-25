
#include <vector>

#include "eigen3/Eigen/Dense"

using Eigen::Vector2f;

#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

namespace global_planner {

class Global_Planner {
 public:

    // Constuctor
    explicit Global_Planner(const std::string& map_file, int type);

    // Sets the global navigation goal at the provided global coordinates
    void SetGlobalNavGoal(Eigen::Vector2f loc);

    // Returns the local coordinates of the intermediate goal along the global path
    Eigen::Vector2f GetLocalNavGoal(Vector2f veh_loc, float veh_angle);

    // Computes the global path from the vehicle's pose to the global nav goal
    void ComputeGlobalPath(Vector2f veh_loc, float veh_angle);

    // Updates the path index based on the current location of the vehicle
    void UpdatePathIndex(Vector2f veh_loc, float veh_angle);

    // Checks whether the global path is still valid, recomputing the path if not
    void CheckPathValid(Vector2f veh_loc, float veh_angle);

    // Plots the global path plan to the provided visualization message
    void PlotGlobalPathVis(amrl_msgs::VisualizationMsg& vis_msg);

    // Plots the local path plan to the provided visualization message
    void PlotLocalPathVis(amrl_msgs::VisualizationMsg& vis_msg);

 private:

    // Map of the environment.
    vector_map::VectorMap map_;

    // Global search type: 0 = ..., 1 = ...
    int type_;

    // Global coordinates for the navigation goal
    Eigen::Vector2f global_nav_goal_;

    // Distance tom look ahead on global path for selecting the local goal
    float local_nav_dist_;

    // Global path - sequence of points
    vector<Eigen::Vector2f> global_path_;

    // Index of nearest global path point to the vehicle - let's us restrict our search to only nearby path points
    int path_index_;

}

}  // namespace global_planner

#endif  // GLOBAL_PLANNER_H
