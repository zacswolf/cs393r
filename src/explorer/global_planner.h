
#include <vector>
#include "eigen3/Eigen/Dense"
#include "simple_queue.h"
#include "visualization/visualization.h"
#include "vector_map/vector_map.h"
#include <unordered_set>
#include "vector_hash.h"
#include "evidence_grid.h"

using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Vector2i;
using Eigen::Vector3i;
using vector_map::VectorMap;


#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

class Global_Planner {
public:
    bool at_path_end_;

    const float NUM_PIXELS_GOAL = 5.;
    const int NUM_ANGLES = 8;
    const float raster_pixel_size;

    const float x_min;
    const float x_max;
    const float y_min;
    const float y_max;
    const int num_x;
    const int num_y;

    enum class PathAlgo {
        Simple = 0,
        Complex = 1,
        Mixed = 2
    };
    const PathAlgo path_algo_;
    const int mixed_lookahead = 50;

    EvidenceGrid& evidence_grid_;

    // Constuctor
    explicit Global_Planner(EvidenceGrid& evidence_grid, float raster_pixel_size, int x_min, int x_max, int y_min, int y_max);

    // Sets the global navigation goal at the provided global coordinates
    void SetGlobalNavGoal(Vector2f loc);

    // Returns the local coordinates of the intermediate goal along the global path
    Vector3f GetLocalNavGoal(Vector2f veh_loc, float veh_angle);

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

private:

    struct GridPt {
        Eigen::Vector4i parent = Eigen::Vector4i(-1, -1, -1, -1); //TODO FIX
        float cost = std::numeric_limits<float>::max();
    };

    struct GridPtSimple {
        Vector2i parent = Vector2i(-1, -1);
        float cost = std::numeric_limits<float>::max();
    };


    // Map grid
    std::vector<Eigen::Matrix<GridPt, -1, -1>> grid_;

    // Simple map grid
    Eigen::Matrix<GridPtSimple, -1, -1> simple_grid_;

    // Wall grid: 0 = open, 1 = wall, 2 = padding

    struct WallStruct {
        bool is_wall;
        int padding_counter;
    };


    Eigen::Matrix<WallStruct, -1, -1> wall_grid_;



public:
    // Converts a map point to a grid point
    Vector2i pointToGrid(Vector2f pt);

    // Converts a map point to a grid point
    Vector2f gridToPoint(Vector2i grid_pt);

    // Update wall grid
    void updateWallGrid(std::unordered_set<Vector2i, matrix_hash<Vector2i>> new_walls, std::unordered_set<Vector2i, matrix_hash<Vector2i>> deleted_walls, bool soft_update);
private:

    // A-Star path algos
    void SimpleAStar(Vector2i veh_loc, Vector2i goal_loc);
    void ComplexAStar(Vector2i veh_loc, float veh_angle, Vector2i goal_loc);



    // Return all neighbors to a grid point
    std::vector<Eigen::Vector4i> GetComplexNeighbors(Eigen::Vector4i current);
    std::vector<Eigen::Vector2i> GetSimpleNeighbors(Eigen::Vector2i current);


    // Map of the environment.
    vector_map::VectorMap map_;

    // Global search type: 0 = ..., 1 = ...
    int type_;

    // Global coordinates for the navigation goal
    Vector2f global_nav_goal_;

    // Local coordinates for the temporary goal
    Vector3f local_nav_goal_;

    // Distance tom look ahead on global path for selecting the local goal
    float local_nav_dist_;

    // Global path - sequence of points
    std::vector<Vector3f> global_path_;

    // Simple global path
    std::vector<Vector2f> simple_global_path_;

    // Index of nearest global path point to the vehicle - let's us restrict our search to only nearby path points
    int path_index_;

    // is ready
    bool is_ready_;
};

#endif  // GLOBAL_PLANNER_H
