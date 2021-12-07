
#include <vector>
#include "eigen3/Eigen/Dense"
#include "simple_queue.h"
#include "visualization/visualization.h"
#include "vector_map/vector_map.h"

#include "evidence_grid.h"
#include <queue>

using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;
using std::vector;


#ifndef FRONTIER_H
#define FRONTIER_H

class Frontier {
 public:
    explicit Frontier() {
        
    }
    
    Vector2f findFrontier(EvidenceGrid evidence_grid, Vector2f robot_loc) {//, amrl_msgs::VisualizationMsg& vis_msg) {

        Vector2i grid_loc = evidence_grid.pointToGrid(robot_loc);

        const static Eigen::Vector2i offsets[] = {
            Vector2i(-1,0),
            Vector2i(0,1),
            Vector2i(1,0),
            Vector2i(0,-1)
        };

        const static Eigen::Vector2i big_offsets[] = {
            Eigen::Vector2i(-1, -1),
            Eigen::Vector2i(-1, 0),
            Eigen::Vector2i(-1, 1),
            Eigen::Vector2i(1, -1),
            Eigen::Vector2i(1, 0),
            Eigen::Vector2i(1, 1),
            Eigen::Vector2i(0, -1),
            Eigen::Vector2i(0, 0),
            Eigen::Vector2i(0, 1),
            Eigen::Vector2i(0, -2),
            Eigen::Vector2i(0, 2),
            Eigen::Vector2i(-2, 0),
            Eigen::Vector2i(2, 0),
            //Eigen::Vector2i(-3, 0),
            Eigen::Vector2i(-2, 1),
            Eigen::Vector2i(-1, 2),
            //Eigen::Vector2i(0, 3),
            Eigen::Vector2i(1, 2),
            Eigen::Vector2i(2, 1),
            //Eigen::Vector2i(3, 0),
            Eigen::Vector2i(-2, -1),
            Eigen::Vector2i(-1, -2),
            //Eigen::Vector2i(0, -3),
            Eigen::Vector2i(1, -2),
            Eigen::Vector2i(2, -1),
        };

        Eigen::Matrix<bool,-1, -1> visited = Eigen::Matrix<bool,-1,-1>::Constant(evidence_grid.num_x, evidence_grid.num_y, false);
        std::queue<Vector2i> queue;
        if (!(evidence_grid.evidence_grid_(grid_loc[0], grid_loc[1]) < .5)) {
            std::cout << "<Frontier finder> Starting loc is not open!\n";
            return evidence_grid.gridToPoint(grid_loc);
        }
        visited(grid_loc[0], grid_loc[1]) = true;
        queue.push(grid_loc);

        while (!queue.empty()) {
            Vector2i current = queue.front();
            queue.pop();
            // Iterate over neighbors
            bool neighbors_wall = false;
            for (Vector2i offset : big_offsets) {
                Vector2i neighbor = current + offset;
                if (evidence_grid.evidence_grid_(neighbor[0], neighbor[1]) > .5) {
                    neighbors_wall = true;
                }
            }
            if (!neighbors_wall) {
                for (Vector2i offset : offsets) {
                    Vector2i neighbor = current + offset;
                    if (!(visited(neighbor[0], neighbor[1]))) {
                        // Eigen::Vector2f pt = evidence_grid.gridToPoint(neighbor);
                        // Check if unknown, open, or obstructed
                        if (evidence_grid.evidence_grid_(neighbor[0], neighbor[1]) < .5) {
                            // Open
                            queue.push(neighbor);
                            //visualization::DrawCross(pt, 0.1, 0xFF0000, vis_msg);
                        } else if (evidence_grid.evidence_grid_(neighbor[0], neighbor[1]) == .5) {
                            // Unknown
                            if (!(evidence_grid.evidence_grid_(current[0], current[1]) < .5)) {
                                std::cout << "<Frontier finder> Found frontier point but from a non open point\n";
                            }
                            // visualization::DrawCross(pt, 0.1, 0x0000FF, vis_msg);
                            return evidence_grid.gridToPoint(current);
                        } else {
                            // Wall
                            // visualization::DrawCross(pt, 0.1, 0x00FF00, vis_msg);
                        }

                        visited(neighbor[0], neighbor[1]) = true;
                    }
                }
            }
        }
        std::cout << "<Frontier finder> Did not find frontier point\n";
        return evidence_grid.gridToPoint(grid_loc);
    }

};

#endif  // FRONTIER_H
