

#include "global_planner.h"
#include "shared/math/line2d.h"
#include "unordered_map"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using std::vector;
using geometry::line2f;


DEFINE_double(raster_pixel_size, 0.1, "Distance between pixels in the raster");

// Constuctor
Global_Planner::Global_Planner(const std::string& map_file) {

   // Load map file
   map_.Load(map_file);

   // Turn Map into grid
   RasterizeMap();

   Type type = Type::A_star;
   // Initialize 
   switch(type) {
      case A_star: 
         // A star init right here
         
         break;
      default:
         std::cout << "Global_Planner Type error\n"; 
   }

   global_path_ = std::vector<Eigen::Vector2f>();

}

void Global_Planner::RasterizeMap() {
   x_min = 100000;
   x_max = -100000;
   
   y_min = 100000;
   y_max = -100000;

   for (line2f& map_line : map_.lines) {
      float xs[] = {map_line.p0[0], map_line.p1[0]};
      for (float x : xs) {
         x_min = (x < x_min)? x : x_min;
         x_max = (x > x_max)? x : x_max;
      }

      float ys[] = {map_line.p0[1], map_line.p1[1]};
      
      for (float y : ys) {
         y_min = (y < y_min)? y : y_min;
         y_max = (y > y_max)? y : y_max;
      }
   }
   std::cout << "(" << x_min << ", " << y_min << ") x (" << x_max << ", " << y_max << ")" << std::endl;
   
   const int num_x = (x_max - x_min) / FLAGS_raster_pixel_size;
   const int num_y = (y_max - y_min) / FLAGS_raster_pixel_size;
   std::cout << "Grid Size: " << num_x << " x " << num_y << "\n\n";
   //false, Eigen::Vector2i(-1, -1), std::numeric_limits<float>::max()
   grid_ = Eigen::Matrix<GridPt,-1,-1>::Constant(num_x, num_y, GridPt{});
   //std::cout << "Is wall? (should be 0 or false) : " << grid_(0,0).is_wall << "\n\n";
   
   // Iterate over all lines in the map, mark obstacles with a 1
   for (size_t j = 0; j < map_.lines.size(); j++) {
      const line2f map_line = map_.lines[j];
      Vector2f line_unit_normal = map_line.Dir();
      Vector2f start_pt = map_line.p0;
      
      for (float walk = 0; walk < map_line.Length(); walk += FLAGS_raster_pixel_size/2.) {
         Vector2f walk_pt = (line_unit_normal * walk) + start_pt;
         
         // std::cout << "APPLE\n";
         Eigen::Vector2i grid_xy = pointToGrid(walk_pt);
         // std::cout << "ORANGE: " << grid_xy[0] << ", " << grid_xy[1] << "\t"<< walk_pt[0] << ", " << walk_pt[1] << std::endl;

         Eigen::Vector2i offsets[] = {
            Eigen::Vector2i(-1, -1),
            Eigen::Vector2i(-1, 0),
            Eigen::Vector2i(-1, 1),
            Eigen::Vector2i(1, -1),
            Eigen::Vector2i(1, 0),
            Eigen::Vector2i(1, 1),
            Eigen::Vector2i(0, -1),
            Eigen::Vector2i(0, 0),
            Eigen::Vector2i(0, 1)
         };
         
         // std::cout << "current loc is wall " << grid_(current[0], current[1]).is_wall << std::endl;
         for (Eigen::Vector2i offset : offsets){
            Eigen::Vector2i offset_pt = offset + grid_xy;
            if (offset_pt[0] >= 0 && offset_pt[0] < num_x && offset_pt[1] >= 0 && offset_pt[1] < num_y) {
               grid_(offset_pt[0], offset_pt[1]).is_wall = true;
            }
            
         }
         // grid_(grid_xy[0], grid_xy[1]).is_wall = true;
         // grid_(grid_xy[0]-1, grid_xy[1]).is_wall = true;
         // grid_(grid_xy[0]+1, grid_xy[1]).is_wall = true;
         // grid_(grid_xy[0], grid_xy[1]-1).is_wall = true;
         // grid_(grid_xy[0]-1, grid_xy[1]-1).is_wall = true;
         // grid_(grid_xy[0]+1, grid_xy[1]-1).is_wall = true;
         // grid_(grid_xy[0], grid_xy[1]+1).is_wall = true;
         // grid_(grid_xy[0]-1, grid_xy[1]+1).is_wall = true;
         // grid_(grid_xy[0]+1, grid_xy[1]+1).is_wall = true;
      }
   }   
}

// Sets the global navigation goal at the provided global coordinates
void Global_Planner::SetGlobalNavGoal(Eigen::Vector2f loc) {
   global_nav_goal_ = loc;
}

// Returns the local coordinates of the intermediate goal along the global path
Eigen::Vector2f Global_Planner::GetLocalNavGoal(Vector2f veh_loc, float veh_angle) {
   return Eigen::Vector2f(0,0);
}

// Computes the global path from the vehicle's pose to the global nav goal
void Global_Planner::ComputeGlobalPath(Vector2f veh_loc, float veh_angle) {
   
   Eigen::Vector2i goal = pointToGrid(global_nav_goal_);

   SimpleQueue<Eigen::Vector2i, float> pri_queue;

   const Eigen::Vector2i start = pointToGrid(veh_loc);
   
   pri_queue.Push(start, 0); // Push initial location
   
   // std::unordered_map<Eigen::Vector2i, Eigen::Vector2i> parent;
   // std::unordered_map<Eigen::Vector2i, float> cost;

   // cost[loc] = 0;
   // parent[loc] = nullptr;
   for (int i = 0; i < grid_.rows(); i++) {
      for (int j = 0; j < grid_.cols(); j++) {
         grid_(i, j).cost = std::numeric_limits<float>::max();
      }
   }

   grid_(start[0], start[1]).cost = 0;

   


   // Begin A*
   Eigen::Vector2i current;
   while (!pri_queue.Empty()) {
      current = pri_queue.Pop();
      if (current == goal) {
         break;
      } else {
         vector<Eigen::Vector2i> neighbors = GetNeighbors(current);
         for (Eigen::Vector2i& neighbor : neighbors) {
            // Eigen::Vector2f step_dist =  .cast <int> ();
            float edge_cost = (neighbor - current).cast<float>().norm();
            float new_cost = grid_(current[0], current[1]).cost + edge_cost;
            if (new_cost < grid_(neighbor[0], neighbor[1]).cost) {
               grid_(neighbor[0], neighbor[1]).cost = new_cost;
               //Eigen::Vector2i dist_to_goal = goal - neighbor;
               float heuristic = (neighbor - goal).cast<float>().norm(); // Euclidean distance
               //float heuristic = abs(dist_to_goal[0]) + abs(dist_to_goal[1]); // Manhattan distance
               pri_queue.Push(neighbor, new_cost + heuristic);
               grid_(neighbor[0], neighbor[1]).parent = current;
            }  
         }
      }
   }

   if (current != goal) {
      std::cout << "[ERROR] A* could not find a path to the goal!\n\n";
      return;
   }

   // Store solution (convert unordered map to vector)
   global_path_.clear();
   global_path_.push_back(gridToPoint(goal));

   Eigen::Vector2i pt = goal;
   while (grid_(pt[0], pt[1]).parent != start) {
      global_path_.push_back(gridToPoint(grid_(pt[0], pt[1]).parent));
      pt = grid_(pt[0], pt[1]).parent;
   }

   std::reverse(global_path_.begin(), global_path_.end());
}

// Updates the path index based on the current location of the vehicle
void Global_Planner::UpdatePathIndex(Vector2f veh_loc, float veh_angle) {

}

// Checks whether the global path is still valid, recomputing the path if not
void Global_Planner::CheckPathValid(Vector2f veh_loc, float veh_angle) {

}

// Plots the global path plan to the provided visualization message
void Global_Planner::PlotGlobalPathVis(amrl_msgs::VisualizationMsg& vis_msg) {

   for (Eigen::Vector2f pt : global_path_) {
      visualization::DrawCross(pt, 0.05, 0x000000, vis_msg);
   }

   // for (int ii = 0; ii < grid_.rows(); ii++) {
   //    for (int jj = 0; jj < grid_.cols(); jj++) {
         
   //       if (grid_(ii,jj).is_wall) {
   //          Eigen::Vector2f pt = gridToPoint(Eigen::Vector2i(ii,jj));
   //          visualization::DrawCross(pt, 0.02, 0x512565, vis_msg);
   //       }

   //    }
   // }

}

// Plots the local path plan to the provided visualization message
void Global_Planner::PlotLocalPathVis(amrl_msgs::VisualizationMsg& vis_msg) {

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

vector<Eigen::Vector2i> Global_Planner::GetNeighbors(Eigen::Vector2i current) {
   vector<Eigen::Vector2i> neighbors = vector<Eigen::Vector2i>();
   
   Eigen::Vector2i pot_neighbors[] = {
      Eigen::Vector2i(-1, -1),
      Eigen::Vector2i(-1, 0),
      Eigen::Vector2i(-1, 1),
      Eigen::Vector2i(1, -1),
      Eigen::Vector2i(1, 0),
      Eigen::Vector2i(1, 1),
      Eigen::Vector2i(0, -1),
      Eigen::Vector2i(0, 1)
   };
   
   // std::cout << "current loc is wall " << grid_(current[0], current[1]).is_wall << std::endl;
   for (Eigen::Vector2i neighbor : pot_neighbors){
      Eigen::Vector2i nei = neighbor + current;
      if (!grid_(nei[0], nei[1]).is_wall) {
         neighbors.push_back(nei);
      }
      
   }

   return neighbors;
}

