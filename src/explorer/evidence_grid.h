
#include <vector>
#include "eigen3/Eigen/Dense"
#include "simple_queue.h"
#include "visualization/visualization.h"
#include "vector_map/vector_map.h"

#include <unordered_set>
#include "vector_hash.h"


using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;
using std::vector;


#ifndef EVIDENCE_GRID_H
#define EVIDENCE_GRID_H

class EvidenceGrid {
 public:
  Eigen::Vector2i pointToGrid(Eigen::Vector2f pt) {
    Eigen::Vector2f temp_pt = ((pt-Vector2f(x_min, y_min)).array() / raster_pixel_size).round();
    Eigen::Vector2i cast_pt = temp_pt.cast <int> ();
    return cast_pt;
  }

  Eigen::Vector2f gridToPoint(Eigen::Vector2i grid_pt) {
    Eigen::Vector2f cast_grid_pt = grid_pt.cast <float> ();
    return cast_grid_pt * raster_pixel_size + Eigen::Vector2f(x_min, y_min);
  }
  
  float raster_pixel_size;
  float x_min;
  float x_max;
  float y_min;
  float y_max;
  float num_x;
  float num_y;
  Eigen::Matrix<float, -1, -1> evidence_grid_;
  
  explicit EvidenceGrid() {

    // Change these values in global_planner.cc too!
    raster_pixel_size = 0.1;
    x_min = -60;
    x_max = 60;
    
    y_min = -60;
    y_max = 60;

    num_x = (x_max - x_min) / raster_pixel_size;
    num_y = (y_max - y_min) / raster_pixel_size;

    evidence_grid_ = Eigen::Matrix<float,-1,-1>::Constant(num_x, num_y, 0.5);

    for (int x = -10; x <= 0; x++) {
      for (int y = -6; y <= 6; y++) {
        evidence_grid_((num_x/2) + x, (num_y/2) + y) = 0.; //TODO: do bayesian update
      }
    }
  }

  void UpdateEvidenceGrid(std::vector<Eigen::Vector2f> new_points, std::vector<Eigen::Vector2f> new_points_open, Eigen::Vector2f robot_loc, float robot_angle, std::unordered_set<Vector2i, matrix_hash<Eigen::Vector2i>> &new_walls, bool soft_update) {
    Eigen::Vector2i grid_robot_loc = pointToGrid(robot_loc);
    //Eigen::Vector2i grid_robot_loc = pointToGrid(Eigen::Vector2f(0,0));
    //std::cout << grid_robot_loc.transpose() << "\n";
    
    for (Vector2f& point : new_points) {
      plotLine(grid_robot_loc, point, false, new_walls);
    }

    for (Vector2f& point : new_points_open) {
      plotLine(grid_robot_loc, point, true, new_walls);
    }
  }

    // Plots the evidence grid
    void PlotEvidenceVis(amrl_msgs::VisualizationMsg& vis_msg) {
      for (int x = 0; x < num_x; x++) {
        for (int y = 0; y < num_y; y++) {
          float evidence = evidence_grid_(x,y);
          // if (evidence < 0.5) {
          //   // Plot unexplored
          //   Eigen::Vector2f pt = gridToPoint(Eigen::Vector2i(x,y));
          //   visualization::DrawCross(pt, 0.05, 0x3333BB, vis_msg);
          // }
          if (evidence > 0.5) {
            // Plot obstructed
            Eigen::Vector2f pt = gridToPoint(Eigen::Vector2i(x,y));
            visualization::DrawCross(pt, 0.05, 0xa903fc, vis_msg);
          }
        }
      }

    }

    // Plots the evidence grid
    void PlotNeighborsVis(Eigen::Vector2f frontier_point, amrl_msgs::VisualizationMsg& vis_msg) {

      Eigen::Vector2i frontier_grid = pointToGrid(frontier_point);

      const static Eigen::Vector2i offsets[] = {
          Vector2i(-1,0),
          Vector2i(0,1),
          Vector2i(1,0),
          Vector2i(0,-1)
      };

      for (Vector2i offset : offsets) {
        Vector2i neighbor = frontier_grid + offset;
        if (evidence_grid_(neighbor[0], neighbor[1]) < 0.5) {
          visualization::DrawCross(gridToPoint(neighbor), 0.05, 0xFF0000, vis_msg);
        }
        if (evidence_grid_(neighbor[0], neighbor[1]) > 0.5) {
          visualization::DrawCross(gridToPoint(neighbor), 0.05, 0x00FF00, vis_msg);
        }
        if (evidence_grid_(neighbor[0], neighbor[1]) == 0.5) {
          visualization::DrawCross(gridToPoint(neighbor), 0.05, 0x0000FF, vis_msg);
        }
      }

    }

  bool is_known(Vector2i loc) {
    return evidence_grid_(loc[0], loc[1]) < 0.5;
  }

 private:

  void plotLineLow(int x0, int y0, int x1, int y1, bool isOpen) {
    //std::cout << "Low\n";
    int dx = x1 - x0;
    int dy = y1 - y0;
    int yi = 1;
    if (dy < 0) {
      yi = -1;
      dy = -dy;
    }
    int D = (2 * dy) - dx;
    int y = y0;

    for (int x = x0; x <= x1; x++) {
      // Add to evidence grid, TODO: bayesian update
      if (evidence_grid_(x,y) <= 0.5) {
        evidence_grid_(x,y) = 0.;
      }
      if (evidence_grid_(x,y+1) <= 0.5) {
        evidence_grid_(x,y+1) = 0.;
      }

      if (D > 0) {
        y = y + yi;
        D = D + (2 * (dy - dx));
      } else {
        D = D + 2*dy;
      }
    }
  }

  void plotLineHigh(int x0, int y0, int x1, int y1, bool isOpen) {
    //std::cout << "High --  (" << x0 << ", " << y0 << ") to (" << x1 << ", " << y1 << ")\n";
    int dx = x1 - x0;
    int dy = y1 - y0;
    int xi = 1;
    if (dx < 0) {
      xi = -1;
      dx = -dx;
    }
    int D = (2 * dx) - dy;
    int x = x0;

    for (int y = y0; y <= y1; y++) {
      // Add to evidence grid, TODO: bayesian update
      if (evidence_grid_(x,y) <= 0.5) {
        evidence_grid_(x,y) = 0.;
      }
      if (evidence_grid_(x+1,y) <= 0.5) {
        evidence_grid_(x+1,y) = 0.;
      }

      if (D > 0) {
        x = x + xi;
        D = D + (2 * (dx - dy));
      } else {
        D = D + 2*dx;
      }
    }
  }
  
  void plotLine(Vector2i grid_robot_loc, Vector2f point, bool isOpen, std::unordered_set<Vector2i, matrix_hash<Eigen::Vector2i>> &new_walls) {
    int x0 = grid_robot_loc[0];
    int y0 = grid_robot_loc[1];
    
    Eigen::Vector2i grid_pt_loc = pointToGrid(point);

    int x1 = grid_pt_loc[0];
    int y1 = grid_pt_loc[1];

    //std::cout << "(" << x0 << "," << y0 << ") to (" << x1 << "," << y1 << ")\n";

    if (abs(y1 - y0) < abs(x1 - x0)) {
      if (x0 > x1) {
        plotLineLow(x1, y1, x0, y0, isOpen);
      } else {
        plotLineLow(x0, y0, x1, y1, isOpen);
      }
    } else {
      if (y0 > y1) {
        plotLineHigh(x1, y1, x0, y0, isOpen);
      } else {
        plotLineHigh(x0, y0, x1, y1, isOpen);
      }
    }

    if (!isOpen){
      evidence_grid_(x1,y1) = 1.;
      new_walls.insert(Eigen::Vector2i(x1,y1));
    } else {
    // Add to evidence grid, TODO: bayesian update
    // if (evidence_grid_(x1,y1) <= 0.5) {
    //   evidence_grid_(x1,y1) = 0.;
    // }
    }
  }

  // void update_evidence(int x, int y );
};

#endif  // EVIDENCE_GRID_H
