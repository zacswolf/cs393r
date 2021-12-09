

#include "global_planner.h"
#include "shared/math/line2d.h"
#include "unordered_map"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using Eigen::Vector3f;
using std::vector;
using geometry::line2f;


// HARD CODED IN EVIDENCE GRID TODO: FIX
DEFINE_double(raster_pixel_size, 0.1, "Distance between pixels in the raster");

DEFINE_int32(num_angles, 8, "Number of angles to use");

// Constuctor
Global_Planner::Global_Planner(): at_path_end_(false) {

   this->is_ready_ = false;


   // Turn Map into grid
   grid_ = std::vector<Eigen::Matrix<GridPt,-1,-1>>(FLAGS_num_angles);
   
   x_min = -50;
   x_max = 50;
   
   y_min = -50;
   y_max = 50;

   num_x = (x_max - x_min) / FLAGS_raster_pixel_size;
   num_y = (y_max - y_min) / FLAGS_raster_pixel_size;

   simple_grid_ = Eigen::Matrix<GridPtSimple,-1,-1>::Constant(num_x, num_y, GridPtSimple{});
   fill(grid_.begin(), grid_.end(), Eigen::Matrix<GridPt,-1,-1>::Constant(num_x, num_y, GridPt{}));
   wall_grid_ = Eigen::Matrix<int,-1,-1>::Constant(num_x, num_y, 0);
   std::cout << "Map created\n";

   global_path_ = std::vector<Eigen::Vector3f>();
   simple_global_path_ = std::vector<Eigen::Vector2f>();
}

// Sets the global navigation goal at the provided global coordinates
void Global_Planner::SetGlobalNavGoal(Eigen::Vector2f loc) {
   global_nav_goal_ = loc;
   this->is_ready_ = true;
}

// Returns the local coordinates of the intermediate goal along the global path
Eigen::Vector3f Global_Planner::GetLocalNavGoal(Vector2f veh_loc, float veh_angle) {
   Eigen::Vector3f local_goal;

   if (global_path_.size() == 0) {
      return Eigen::Vector3f(0.,0.,0.);
   }

   UpdatePathIndex(veh_loc, veh_angle);

   local_goal = global_path_[std::min(path_index_ + 1, (int)global_path_.size()-1)];

   Vector2f local_goal_2f = local_goal.segment(0,2);

   at_path_end_ = path_index_ >= ((int)global_path_.size()-1) - 3;

   Eigen::Rotation2Df r(-veh_angle);
   local_goal_2f = r * (local_goal_2f - veh_loc);
   local_goal_2f = local_goal_2f.array() / local_goal_2f.norm();

   bool going_backward = (global_path_[path_index_][2] > 0);
   bool going_backward_lookahead = (global_path_[path_index_ + 1][2] > 0);
   if (going_backward != going_backward_lookahead) {
      local_goal_2f[1] *= -1;
   }
   local_goal[2] = going_backward;
   
   local_goal[0] = local_goal_2f[0];
   local_goal[1] = local_goal_2f[1];
   local_nav_goal_ = local_goal;
   return local_goal;
   
}

// Computes the global path from the vehicle's pose to the global nav goal
void Global_Planner::ComputeGlobalPath(Vector2f veh_loc, float veh_angle) {

   Eigen::Vector2i goal_2d = pointToGrid(global_nav_goal_);
   Eigen::Vector3i goal = Eigen::Vector3i(goal_2d[0], goal_2d[1], 0);

   // 
   // Compute simple path
   // 


   // 
   // Compute lattice planned A* path
   // 


   at_path_end_ = false;

   int rounded_angle_ind = ((int) round(math_util::AngleMod(veh_angle) * FLAGS_num_angles / (2 * M_PI)) );
   rounded_angle_ind = (rounded_angle_ind % NUM_ANGLES + NUM_ANGLES) % NUM_ANGLES;
   std::cout << "Angle Ind: " << rounded_angle_ind << "\n";

   SimpleQueue<Eigen::Vector4i, float> pri_queue;

   Eigen::Vector2i start_2d = pointToGrid(veh_loc);
   const Eigen::Vector4i start = Eigen::Vector4i(start_2d[0], start_2d[1], rounded_angle_ind, 0);
   
   pri_queue.Push(start, 0); // Push initial location
   
   for (int i = 0; i < grid_[0].rows(); i++) {
      for (int j = 0; j < grid_[0].cols(); j++) {
         for (int angle_ind = 0; angle_ind < FLAGS_num_angles; angle_ind++) {
            grid_[angle_ind](i, j).cost = std::numeric_limits<float>::max();
         }
      }
   }

   grid_[start[2]](start[0], start[1]).cost = 0;

   // Begin A*
   std::cout << "<GP A*> Beginning A*\n";
   Eigen::Vector4i current;
   while (!pri_queue.Empty()) {
      current = pri_queue.Pop();
      Vector2f rel_to_goal = current.cast<float>().segment(0,2) - goal.cast<float>().segment(0,2);
      if (rel_to_goal.norm() < NUM_PIXELS_GOAL) {
         std::cout << rel_to_goal.norm() << "\n\n";
         break;
      } else {
         vector<Eigen::Vector4i> neighbors = GetNeighbors(current);
         for (Eigen::Vector4i& neighbor : neighbors) {
            bool is_backward = neighbor[3];
            float edge_cost = (neighbor.segment(0,2) - current.segment(0,2)).cast<float>().norm();

            // Penalize edge_cost if we have to go backwards
            if (is_backward) {
               edge_cost *= 1024;
            }
            //edge_cost += is_backward * (edge_cost * 100);

            // Neighbors can be in wall padding if current is in wall padding
            // We want to penalize moving within wall padding so we add 10 map pixels to the edge cost
            bool neighbor_in_padding = (wall_grid_(neighbor[0], neighbor[1]) == 2);
            edge_cost += neighbor_in_padding * 10;
            
            float new_cost = grid_[current[2]](current[0], current[1]).cost + edge_cost;
            if (new_cost < grid_[neighbor[2]](neighbor[0], neighbor[1]).cost) {
               grid_[neighbor[2]](neighbor[0], neighbor[1]).cost = new_cost;
               
               // Note no longer optimal with the 1.2
               float heuristic = 1.2 *(neighbor.segment(0,2) - goal.segment(0,2)).cast<float>().norm(); // Euclidean distance
               //float heuristic = abs(dist_to_goal[0]) + abs(dist_to_goal[1]); // Manhattan distance
               
               pri_queue.Push(neighbor, new_cost + heuristic);
               grid_[neighbor[2]](neighbor[0], neighbor[1]).parent = current;
            }  
         }
      }

   }
   std::cout << "<GP A*> Finished A*\n";

   // Store solution (convert unordered map to vector)
   global_path_.clear();
   
   Eigen::Vector2f path_point = gridToPoint(current.segment(0,2));
   global_path_.push_back(Eigen::Vector3f(path_point[0], path_point[1], current[3]));

   if (current != start) { //TODO: remove

      Eigen::Vector4i loc = current;
      Eigen::Vector4i parent_loc = grid_[loc[2]](loc[0], loc[1]).parent;

      bool done = false;
      while (!done) {
         
         parent_loc = grid_[loc[2]](loc[0], loc[1]).parent;
         // old_ang = new_ang;
         // new_ang = parent_pt[2];
         Vector2f parent_point = gridToPoint(parent_loc.segment(0,2));
         Vector2f loc_point = gridToPoint(loc.segment(0,2));
         
         Vector3f new_point = Vector3f(parent_point[0], parent_point[1], parent_loc[3]);
         Vector3f old_point = Vector3f(loc_point[0], loc_point[1], loc[3]);
         
         if ((new_point - old_point).segment(0,2).norm() > 0.15) {
            // Far point, interpolate
            for (float step = 0.125; step < 1.001; step += 0.125) {
               Vector3f interp_point = Vector3f(0,0,0);
               interp_point[0] = new_point[0]*step + old_point[0]*(1-step);
               interp_point[1] = new_point[1]*step + old_point[1]*(1-step);
               interp_point[2] = old_point[2];

               global_path_.push_back(interp_point);
            }
         } else {
            // Close point, no interpolation
            Vector3f mod_new_point = new_point;
            mod_new_point[2] = old_point[2];
            global_path_.push_back(mod_new_point);
         }
         if (grid_[loc[2]](loc[0], loc[1]).parent == start) {
            done = true;
         } else {
            loc = grid_[loc[2]](loc[0], loc[1]).parent;
         }
      }

      std::reverse(global_path_.begin(), global_path_.end());

   }

   this->path_index_ = 0;
}

// Updates the path index based on the current location of the vehicle
void Global_Planner::UpdatePathIndex(Vector2f veh_loc, float veh_angle) {

   if (path_index_ < (int)global_path_.size() - 1) {
      float dist_to_current_point = (global_path_[path_index_].segment(0,2) - veh_loc).norm();
      float dist_to_next_point = (global_path_[path_index_ + 1].segment(0,2) - veh_loc).norm();
      if (dist_to_next_point < dist_to_current_point) {
         path_index_++;
      }

   }

}

// Checks whether the global path is still valid, recomputing the path if not
void Global_Planner::CheckPathValid(Vector2f veh_loc, float veh_angle) {

   if ((global_path_[path_index_].segment(0,2) - veh_loc).norm() > 2.) {
      ComputeGlobalPath(veh_loc, veh_angle);
   }

}

// Plots the global path plan to the provided visualization message
void Global_Planner::PlotWallVis(amrl_msgs::VisualizationMsg& vis_msg) {
   //const int num_x = (x_max - x_min) / FLAGS_raster_pixel_size;
   //const int num_y = (y_max - y_min) / FLAGS_raster_pixel_size;
   for (int x = 0; x < num_x; x++) {
      for (int y = 0; y < num_y; y++) {
         bool is_wall = wall_grid_(x, y) == 1;
         if (is_wall) {
            // Plot wall
            Eigen::Vector2f pt = gridToPoint(Eigen::Vector2i(x,y));
            visualization::DrawCross(pt, 0.1, 0x000000, vis_msg);
         }
      }
   }

}

// Plots the global path plan to the provided visualization message
void Global_Planner::PlotGlobalPathVis(amrl_msgs::VisualizationMsg& vis_msg) {

   for (Eigen::Vector3f pt : global_path_) {
      if (pt[2]) {
         visualization::DrawCross(pt.segment(0,2), 0.05, 0xAA0000, vis_msg);
      } else {
         visualization::DrawCross(pt.segment(0,2), 0.05, 0x000000, vis_msg);
      }
   }

}

// Plots the local path plan to the provided visualization message
void Global_Planner::PlotLocalPathVis(amrl_msgs::VisualizationMsg& vis_msg) {
   visualization::DrawCross(local_nav_goal_.segment(0,2), .5, 0x39B81D, vis_msg);
}

Eigen::Vector2i Global_Planner::pointToGrid(Eigen::Vector2f pt) {
   Eigen::Vector2f temp_pt = ((pt-Vector2f(x_min, y_min)).array() / FLAGS_raster_pixel_size).round();
   Eigen::Vector2i cast_pt = temp_pt.cast <int> ();
   return cast_pt;
}

Eigen::Vector2f Global_Planner::gridToPoint(Eigen::Vector2i grid_pt) {
   Eigen::Vector2f cast_grid_pt = grid_pt.cast <float> ();
   return cast_grid_pt * FLAGS_raster_pixel_size + Eigen::Vector2f(x_min, y_min);
}

std::vector<Eigen::Vector4i> Global_Planner::GetNeighbors(Eigen::Vector4i current) {
   vector<Eigen::Vector4i> neighbors = vector<Eigen::Vector4i>();
   
   int current_ang = current[2];

   // (x, y, angle, is_backward)
   const static Eigen::Vector4i pot_neighbors[8][6] = {
      {
         // 0
         Eigen::Vector4i(1, 0, 0, 0),
         Eigen::Vector4i(10, 5, 1, 0),
         Eigen::Vector4i(10, -5, 7, 0),
         Eigen::Vector4i(-1, 0, 0, 1),
         Eigen::Vector4i(-10, 5, 7, 1),
         Eigen::Vector4i(-10, -5, 1, 1),
      },
      {
         // 1
         Eigen::Vector4i(1, 1, 1, 0),
         Eigen::Vector4i(5, 10, 2, 0),
         Eigen::Vector4i(10, 5, 0, 0),
         Eigen::Vector4i(-1, -1, 1, 1),
         Eigen::Vector4i(-10, -5, 0, 1),
         Eigen::Vector4i(-5, -10, 2, 1)
      },
      {
         // 2
         Eigen::Vector4i(0, 1, 2, 0),
         Eigen::Vector4i(-5, 10, 3, 0),
         Eigen::Vector4i(5, 10, 1, 0),
         Eigen::Vector4i(0, -1, 2, 1),
         Eigen::Vector4i(-5, -10, 1, 1),
         Eigen::Vector4i(5, -10, 3, 1)
      },
      {
         // 3
         Eigen::Vector4i(-1, 1, 3, 0),
         Eigen::Vector4i(-10, 5, 4, 0),
         Eigen::Vector4i(-5, 10, 2, 0),
         Eigen::Vector4i(1, -1, 3, 1),
         Eigen::Vector4i(5, -10, 2, 1),
         Eigen::Vector4i(10, -5, 4, 1)
      },
      {
         // 4
         Eigen::Vector4i(-1, 0, 4, 0),
         Eigen::Vector4i(-10, -5, 5, 0),
         Eigen::Vector4i(-10, 5, 3, 0),
         Eigen::Vector4i(1, 0, 4, 1),
         Eigen::Vector4i(10, -5, 3, 1),
         Eigen::Vector4i(10, 5, 5, 1)
      },
      {
         // 5
         Eigen::Vector4i(-1, -1, 5, 0),
         Eigen::Vector4i(-5, -10, 6, 0),
         Eigen::Vector4i(-10, -5, 4, 0),
         Eigen::Vector4i(1, 1, 5, 1),
         Eigen::Vector4i(10, 5, 4, 1),
         Eigen::Vector4i(5, 10, 6, 1)
      },
      {
         // 6
         Eigen::Vector4i(0, -1, 6, 0),
         Eigen::Vector4i(5, -10, 7, 0),
         Eigen::Vector4i(-5, -10, 5, 0),
         Eigen::Vector4i(0, 1, 6, 1),
         Eigen::Vector4i(5, 10, 5, 1),
         Eigen::Vector4i(-5, 10, 7, 1)
      },
      {
         // 7
         Eigen::Vector4i(1, -1, 7, 0),
         Eigen::Vector4i(10, -5, 0, 0),
         Eigen::Vector4i(5, -10, 6, 0),
         Eigen::Vector4i(-1, 1, 7, 1),
         Eigen::Vector4i(-5, 10, 6, 1),
         Eigen::Vector4i(-10, 5, 0, 1)
      }
   };

   // std::cout << "current loc is wall " << wall_grid_(current[0], current[1]) << std::endl;

   bool in_padding = (wall_grid_(current[0], current[1]) == 2);

   for (Eigen::Vector4i neighbor : pot_neighbors[current_ang]){

      bool is_valid_neighbor = true;

      Eigen::Vector3i cur;
      cur[0] = current[0];
      cur[1] = current[1];

      Eigen::Vector4i nei = neighbor;
      nei[0] += cur[0];
      nei[1] += cur[1];

      float slope = (float)neighbor[1] / (float)neighbor[0];

      if (abs(slope) <= 1.) {
         int incr = (neighbor[0] >= 0) - (neighbor[0] < 0);
         for (int x = 0; abs(x) < abs(neighbor[0]); x += incr) {
            int y = (int) round(slope * (float)x);
            if ((wall_grid_(cur[0] + x, cur[1] + y) == 1) || ((wall_grid_(cur[0] + x, cur[1] + y) == 2) && !in_padding)) { 
               is_valid_neighbor = false;
            }
         }
      } else {
         int incr = (neighbor[1] >= 0) - (neighbor[1] < 0);
         for (int y = 0; abs(y) < abs(neighbor[1]); y += incr) {
            int x = (int) round((float)y / slope);
            if ((wall_grid_(cur[0] + x, cur[1] + y) == 1) || ((wall_grid_(cur[0] + x, cur[1] + y) == 2) && !in_padding)) { 
               is_valid_neighbor = false;
            }
         }
      }

      if (is_valid_neighbor) {
         neighbors.push_back(nei);
      }
   }

   return neighbors;
}

bool Global_Planner::IsReady() {
   return this->is_ready_;
}

void Global_Planner::AddWallsFromSLAM(std::vector<Eigen::Vector2f> point_cloud) {
   const static Eigen::Vector2i offsets[] = {
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

   for (Eigen::Vector2f& point : point_cloud) {
      Eigen::Vector2i wall_index = pointToGrid(point);
   
      for (Eigen::Vector2i offset : offsets){
         Eigen::Vector2i offset_pt = offset + wall_index;
         if (offset_pt[0] >= 0 && offset_pt[0] < num_x && offset_pt[1] >= 0 && offset_pt[1] < num_y) {
            wall_grid_(offset_pt[0], offset_pt[1]) = 1;
         }
      }
   }
   
}

// void updateWallGrid(Eigen::Matrix<float, -1, -1> &evidence_grid) {
//    wall_grid_ = (evidence_grid.array() >= 0.5);
// }

void Global_Planner::updateWallGrid(std::unordered_set<Eigen::Vector2i, matrix_hash<Eigen::Vector2i>> new_walls) {
   // Note: this will go through unknown space
   // Proposed solution: maintain a frontier matrix and when checking for is wall look at both frontier mat and wall_grid
   
   // Other solution: everything starts as a wall, pass in the "free" grid spaces that you want to make not walls,
   // then you check the offsets around each point to see if they're a wall (in the evidence grid) or not, if not then make it open
   
   const static Eigen::Vector2i offsets[] = {
      Eigen::Vector2i(-1, -1),
      Eigen::Vector2i(-1, 0),
      Eigen::Vector2i(-1, 1),
      Eigen::Vector2i(1, -1),
      Eigen::Vector2i(1, 0),
      Eigen::Vector2i(1, 1),
      Eigen::Vector2i(0, -1),
      Eigen::Vector2i(0, 1),
      Eigen::Vector2i(0, -2),
      Eigen::Vector2i(0, 2),
      Eigen::Vector2i(-2, 0),
      Eigen::Vector2i(2, 0),
      Eigen::Vector2i(-3, 0),
      Eigen::Vector2i(-2, 1),
      Eigen::Vector2i(-1, 2),
      Eigen::Vector2i(0, 3),
      Eigen::Vector2i(1, 2),
      Eigen::Vector2i(2, 1),
      Eigen::Vector2i(3, 0),
      Eigen::Vector2i(-2, -1),
      Eigen::Vector2i(-1, -2),
      Eigen::Vector2i(0, -3),
      Eigen::Vector2i(1, -2),
      Eigen::Vector2i(2, -1),
   };


   for (Eigen::Vector2i new_wall : new_walls) {

      if (new_wall[0] >= 0 && new_wall[0] < num_x && new_wall[1] >= 0 && new_wall[1] < num_y) {
         wall_grid_(new_wall[0], new_wall[1]) = 1;
      }

      for (Eigen::Vector2i offset : offsets){
         Eigen::Vector2i offset_pt = offset + new_wall;
         if (offset_pt[0] >= 0 && offset_pt[0] < num_x && offset_pt[1] >= 0 && offset_pt[1] < num_y) {
            if (wall_grid_(offset_pt[0], offset_pt[1]) != 1) {
               wall_grid_(offset_pt[0], offset_pt[1]) = 2;  
            }
         }
      }
   }


   // TODO: add functionality to remove walls
   // Note: non-trivial, will need to add "wall suport" set for each wall and only delete if its empty
}
