
#include <vector>
#include "eigen3/Eigen/Dense"
#include "simple_queue.h"
#include "visualization/visualization.h"
#include "vector_map/vector_map.h"
#include <unordered_set>
#include "vector_hash.h"

using Eigen::Vector2f;
using vector_map::VectorMap;


#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

class Global_Planner {
 public:
  bool at_path_end_;

   const float NUM_PIXELS_GOAL = 5.;
   const int NUM_ANGLES = 8;

   float x_min;
   float y_min;
   float x_max;
   float y_max;
   int num_x;
   int num_y;

    // Constuctor
    explicit Global_Planner();

    // Sets the global navigation goal at the provided global coordinates
    void SetGlobalNavGoal(Eigen::Vector2f loc);

    // Returns the local coordinates of the intermediate goal along the global path
    Eigen::Vector3f GetLocalNavGoal(Vector2f veh_loc, float veh_angle);

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

    void PlotWallVis(amrl_msgs::VisualizationMsg& vis_msg);

    // Check if initialized
    bool IsReady();

   void AddWallsFromSLAM(std::vector<Eigen::Vector2f> point_cloud_);
 private:
 
   struct GridPt {
      Eigen::Vector4i parent = Eigen::Vector4i(-1, -1, -1, -1); //TODO FIX
      float cost = std::numeric_limits<float>::max();
   };
   

   // Map grid
   std::vector<Eigen::Matrix<GridPt, -1, -1>> grid_;

   // Wall grid: 0 = open, 1 = wall, 2 = padding
   Eigen::Matrix<int, -1, -1> wall_grid_;

 public:
   // Converts a map point to a grid point
   Eigen::Vector2i pointToGrid(Eigen::Vector2f pt);

   // Converts a map point to a grid point
   Eigen::Vector2f gridToPoint(Eigen::Vector2i grid_pt);

   // Update wall grid
   void updateWallGrid(std::unordered_set<Eigen::Vector2i, matrix_hash<Eigen::Vector2i>> new_walls);
 private:
   // Return all neighbors to a grid point
   std::vector<Eigen::Vector4i> GetNeighbors(Eigen::Vector4i current);

    // Map of the environment.
    vector_map::VectorMap map_;

    // Global search type: 0 = ..., 1 = ...
    int type_;

    // Global coordinates for the navigation goal
    Eigen::Vector2f global_nav_goal_;

    // Local coordinates for the temporary goal
    Eigen::Vector3f local_nav_goal_;

    // Distance tom look ahead on global path for selecting the local goal
    float local_nav_dist_;

    // Global path - sequence of points
    std::vector<Eigen::Vector3f> global_path_;

    // Index of nearest global path point to the vehicle - let's us restrict our search to only nearby path points
    int path_index_;

    // simple path
    std::vector<Eigen::Vector2f> simple_global_path_;
    int simple_path_index_;
    
    // is ready
    bool is_ready_;

   //  // A* Priority Queue
   // SimpleQueue<Eigen::Vector2i, float> pri_queue;
   
};

#endif  // GLOBAL_PLANNER_H
