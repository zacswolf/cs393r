
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

// DEFINE_double(initial_prob, 0.5, "The probability of an unknown region to be a wall");
// DEFINE_double(prob_wall_given_wall, 0.8, "The probability of a region to be a wall given a wall read");
// DEFINE_double(prob_free_given_free, 0.6, "The probability of a region to be free given a free read");

const double FLAGS_initial_prob = 0.5;
const double FLAGS_prob_wall_given_wall = 0.9;
const double FLAGS_prob_free_given_free = 0.55;
const double FLAGS_prob_max = 0.99;
const double FLAGS_prob_min = 0.01;

class EvidenceGrid {
public:
    Eigen::Vector2i pointToGrid(Eigen::Vector2f pt) {
        Eigen::Vector2f temp_pt = ((pt - Vector2f(x_min, y_min)).array() / raster_pixel_size).round();
        Eigen::Vector2i cast_pt = temp_pt.cast <int>();
        return cast_pt;
    }

    Eigen::Vector2f gridToPoint(Eigen::Vector2i grid_pt) {
        Eigen::Vector2f cast_grid_pt = grid_pt.cast <float>();
        return cast_grid_pt * raster_pixel_size + Eigen::Vector2f(x_min, y_min);
    }

    const float raster_pixel_size;
    const float x_min;
    const float x_max;
    const float y_min;
    const float y_max;
    const float num_x;
    const float num_y;
    Eigen::Matrix<float, -1, -1> evidence_grid_;

    explicit EvidenceGrid(float raster_pixel_size, int x_min, int x_max, int y_min, int y_max) :
        raster_pixel_size(raster_pixel_size), x_min(x_min), x_max(x_max), y_min(y_min), y_max(y_max),
        num_x((x_max - x_min) / raster_pixel_size), num_y((y_max - y_min) / raster_pixel_size) {

        evidence_grid_ = Eigen::Matrix<float, -1, -1>::Constant(num_x, num_y, FLAGS_initial_prob);

        for (int x = -10; x <= 0; x++) {
            for (int y = -6; y <= 6; y++) {
                evidence_grid_((num_x / 2) + x, (num_y / 2) + y) = 0.; //TODO: do bayesian update
            }
        }
    }

    void updateEvidenceGrid(const std::vector<Eigen::Vector2f> new_points, const std::vector<Eigen::Vector2f> new_points_open, Eigen::Vector2f robot_loc, float robot_angle, std::unordered_set<Vector2i, matrix_hash<Eigen::Vector2i>>& new_walls, std::unordered_set<Vector2i, matrix_hash<Eigen::Vector2i>>& deleted_walls, bool soft_update) {
        Eigen::Vector2i grid_robot_loc = pointToGrid(robot_loc);
        //Eigen::Vector2i grid_robot_loc = pointToGrid(Eigen::Vector2f(0,0));
        //std::cout << grid_robot_loc.transpose() << "\n";

        for (const Vector2f& point : new_points) {
            plotLine(grid_robot_loc, point, false, new_walls, deleted_walls);
        }

        for (const Vector2f& point : new_points_open) {
            plotLine(grid_robot_loc, point, true, new_walls, deleted_walls);
        }
    }

    // Plots the evidence grid
    void PlotEvidenceVis(amrl_msgs::VisualizationMsg& vis_msg) {
        for (int x = 0; x < num_x; x++) {
            for (int y = 0; y < num_y; y++) {
                float evidence = evidence_grid_(x, y);
                // if (evidence < FLAGS_initial_prob) {
                //   // Plot unexplored
                //   Eigen::Vector2f pt = gridToPoint(Eigen::Vector2i(x,y));
                //   visualization::DrawCross(pt, 0.05, 0x3333BB, vis_msg);
                // }
                if (evidence > FLAGS_initial_prob) {
                    // Plot obstructed
                    Eigen::Vector2f pt = gridToPoint(Eigen::Vector2i(x, y));
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
            if (evidence_grid_(neighbor[0], neighbor[1]) < FLAGS_initial_prob) {
                visualization::DrawCross(gridToPoint(neighbor), 0.05, 0xFF0000, vis_msg);
            }
            if (evidence_grid_(neighbor[0], neighbor[1]) > FLAGS_initial_prob) {
                visualization::DrawCross(gridToPoint(neighbor), 0.05, 0x00FF00, vis_msg);
            }
            if (evidence_grid_(neighbor[0], neighbor[1]) == FLAGS_initial_prob) {
                visualization::DrawCross(gridToPoint(neighbor), 0.05, 0x0000FF, vis_msg);
            }
        }

    }

    bool is_known(Vector2i loc) {
        return evidence_grid_(loc[0], loc[1]) < FLAGS_initial_prob;
    }

private:

    // returns true if wall to free
    bool update_free(int x, int y) {
        bool was_wall = evidence_grid_(x, y) > FLAGS_initial_prob;
        float measurement_odds = (1 - FLAGS_prob_free_given_free) / FLAGS_prob_free_given_free;
        float odds = (evidence_grid_(x, y) / (1 - evidence_grid_(x, y)));
        float new_odds = measurement_odds * odds;
        evidence_grid_(x, y) = std::max((1. / (1. / new_odds + 1)), FLAGS_prob_min);
        bool now_free = evidence_grid_(x, y) < FLAGS_initial_prob;
        return was_wall && now_free;
    }

    // returns true if free or unknown to wall
    bool update_wall(int x, int y) {
        bool was_not_wall = evidence_grid_(x, y) <= FLAGS_initial_prob;
        float measurement_odds = FLAGS_prob_wall_given_wall / (1 - FLAGS_prob_wall_given_wall);
        float odds = (evidence_grid_(x, y) / (1 - evidence_grid_(x, y)));
        float new_odds = measurement_odds * odds;
        evidence_grid_(x, y) = std::min((1. / (1. / new_odds + 1)), FLAGS_prob_max);
        bool now_wall = evidence_grid_(x, y) > FLAGS_initial_prob;
        return was_not_wall && now_wall;
    }

    void plotLineLow(int x0, int y0, int x1, int y1,
        std::unordered_set<Vector2i, matrix_hash<Eigen::Vector2i>>& deleted_walls) {
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

        for (int x = x0 + 1; x < x1; x++) {
            // Add to evidence grid, TODO: bayesian update
            // Bayes update free
            bool is_new_free = update_free(x, y);
            if (is_new_free) {
                // remove wall
                deleted_walls.insert(Eigen::Vector2i(x, y));
            }

            // if (evidence_grid_(x,y) <= FLAGS_initial_prob) {
            //   evidence_grid_(x,y) = 0.;
            // }
            // if (evidence_grid_(x,y+1) <= FLAGS_initial_prob) {
            //   evidence_grid_(x,y+1) = 0.;
            // }

            if (D > 0) {
                y = y + yi;
                D = D + (2 * (dy - dx));
            } else {
                D = D + 2 * dy;
            }
        }
    }

    void plotLineHigh(int x0, int y0, int x1, int y1,
        std::unordered_set<Vector2i, matrix_hash<Eigen::Vector2i>>& deleted_walls) {
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

        for (int y = y0 + 1; y < y1; y++) {
            // Add to evidence grid, TODO: bayesian update

            bool is_new_free = update_free(x, y);
            if (is_new_free) {
                // remove wall
                deleted_walls.insert(Eigen::Vector2i(x, y));
            }

            // if (evidence_grid_(x,y) <= FLAGS_initial_prob) {
            //   evidence_grid_(x,y) = 0.;
            // }
            // if (evidence_grid_(x+1,y) <= ) {
            //   evidence_grid_(x+1,y) = 0.;
            // }

            if (D > 0) {
                x = x + xi;
                D = D + (2 * (dx - dy));
            } else {
                D = D + 2 * dx;
            }
        }
    }

    void plotLine(Vector2i grid_robot_loc, const Vector2f& point, bool isOpen,
        std::unordered_set<Vector2i, matrix_hash<Eigen::Vector2i>>& new_walls,
        std::unordered_set<Vector2i, matrix_hash<Eigen::Vector2i>>& deleted_walls) {
        int x0 = grid_robot_loc[0];
        int y0 = grid_robot_loc[1];

        Eigen::Vector2i grid_pt_loc = pointToGrid(point);

        int x1 = grid_pt_loc[0];
        int y1 = grid_pt_loc[1];

        //std::cout << "(" << x0 << "," << y0 << ") to (" << x1 << "," << y1 << ")\n";

        if (abs(y1 - y0) < abs(x1 - x0)) {
            if (x0 > x1) {
                plotLineLow(x1, y1, x0, y0, deleted_walls);
            } else {
                plotLineLow(x0, y0, x1, y1, deleted_walls);
            }
        } else {
            if (y0 > y1) {
                plotLineHigh(x1, y1, x0, y0, deleted_walls);
            } else {
                plotLineHigh(x0, y0, x1, y1, deleted_walls);
            }
        }

        if (!isOpen) {
            // evidence_grid_(x1,y1) = 1.;
            bool is_new_wall = update_wall(x1, y1);
            if (is_new_wall) {
                new_walls.insert(Eigen::Vector2i(x1, y1));
            }
        } else {
            // Add to evidence grid, TODO: bayesian update
            // if (evidence_grid_(x1,y1) <= FLAGS_initial_prob) {
            //   evidence_grid_(x1,y1) = 0.;
            // }
        }
    }

};

#endif  // EVIDENCE_GRID_H
