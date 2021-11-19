

#include "global_planner.h"
#include "shared/math/line2d.h"
#include "unordered_map"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using std::vector;
using geometry::line2f;


DEFINE_double(raster_pixel_size, 0.1, "Distance between pixels in the raster");
DEFINE_int32(num_angles, 8, "Number of angles to use");

// Constuctor
Global_Planner::Global_Planner(const std::string& map_file) {

   this->is_ready_ = false;

   // Load map file
   map_.Load(map_file);

   // Turn Map into grid
   RasterizeMap();
   std::cout << "Map rasterized\n"; 

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
   //grid_ = Eigen::Matrix<GridPt,-1,-1>::Constant(num_x, num_y, GridPt{});
   new_grid_ = std::vector<Eigen::Matrix<GridPtNew,-1,-1>>(FLAGS_num_angles);
   fill(new_grid_.begin(), new_grid_.end(), Eigen::Matrix<GridPtNew,-1,-1>::Constant(num_x, num_y, GridPtNew{}));

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
         
         // std::cout << "current loc is wall " << grid_(current[0], current[1]).is_wall << std::endl;
         for (Eigen::Vector2i offset : offsets){
            Eigen::Vector2i offset_pt = offset + grid_xy;
            if (offset_pt[0] >= 0 && offset_pt[0] < num_x && offset_pt[1] >= 0 && offset_pt[1] < num_y) {
               
               for (int angle_ind = 0; angle_ind < FLAGS_num_angles; angle_ind++) {
                  new_grid_[angle_ind](offset_pt[0], offset_pt[1]).is_wall = true;
               }
            }
            
         }
      }
   }   
}

// Sets the global navigation goal at the provided global coordinates
void Global_Planner::SetGlobalNavGoal(Eigen::Vector2f loc) {
   global_nav_goal_ = loc;
   this->is_ready_ = true;
}

// Returns the local coordinates of the intermediate goal along the global path
Eigen::Vector2f Global_Planner::GetLocalNavGoal(Vector2f veh_loc, float veh_angle) {
   Eigen::Vector2f local_goal;

   if (global_path_.size() == 0) {
      return Vector2f(0.,0.);
   }

   UpdatePathIndex(veh_loc, veh_angle);
   //std::cout << path_index_ << "\n";

   local_goal = global_path_[std::min(path_index_ + 4, (int)global_path_.size()-1)];
   // Eigen::Vector2f local_goal_1 = global_path_[std::min(path_index_ + 1, (int)global_path_.size()-1)];
   // Eigen::Vector2f local_goal_2 = global_path_[std::min(path_index_ + 5, (int)global_path_.size()-1)];

   // float dist1 = (local_goal_1 - veh_loc).norm();
   // float dist2 = (local_goal_2 - veh_loc).norm();

   // //std::cout << dist1 << "   " << dist2 << "\n";

   // if (abs(dist1 - 1.5) < abs(dist2 - 1.5)) {
   //    local_goal = local_goal_1;
   // } else {
   //    local_goal = local_goal_2;
   // }

   Eigen::Rotation2Df r(-veh_angle);
   local_goal = r * (local_goal - veh_loc);

   local_nav_goal_ = local_goal;
   
   return local_goal;
   
}

// Computes the global path from the vehicle's pose to the global nav goal
void Global_Planner::ComputeGlobalPath(Vector2f veh_loc, float veh_angle) {
   
   int rounded_angle_ind = ((int) round(math_util::AngleMod(veh_angle) * FLAGS_num_angles / (2 * M_PI)) );
   rounded_angle_ind = (rounded_angle_ind % 8 + 8) % 8;
   std::cout << "Angle Ind: " << rounded_angle_ind << "\n";

   Eigen::Vector2i goal_2d = pointToGrid(global_nav_goal_);
   Eigen::Vector3i goal = Eigen::Vector3i(goal_2d[0], goal_2d[1], 0);

   SimpleQueue<Eigen::Vector3i, float> pri_queue;

   const Eigen::Vector2i start_2d = pointToGrid(veh_loc);
   const Eigen::Vector3i start = Eigen::Vector3i(start_2d[0], start_2d[1], rounded_angle_ind);
   
   pri_queue.Push(start, 0); // Push initial location
   
   for (int i = 0; i < new_grid_[0].rows(); i++) {
      for (int j = 0; j < new_grid_[0].cols(); j++) {
         for (int angle_ind = 0; angle_ind < FLAGS_num_angles; angle_ind++) {
            new_grid_[angle_ind](i, j).cost = std::numeric_limits<float>::max();
         }
      }
   }

   new_grid_[start[2]](start[0], start[1]).cost = 0;

   // Begin A*
   Eigen::Vector3i current;
   while (!pri_queue.Empty()) {
      current = pri_queue.Pop();
      Vector2f rel_to_goal = current.cast<float>().segment(0,2) - goal.cast<float>().segment(0,2);
      if (rel_to_goal.norm() < 3) {
         std::cout << rel_to_goal.norm() << "\n\n";
         break;
      } else {
         vector<Eigen::Vector3i> neighbors = GetNeighborsNew(current);
         for (Eigen::Vector3i& neighbor : neighbors) {
            //Vector2f rel_nei = (neighbor.segment(0,2) - current.segment(0,2));
            float edge_cost = (neighbor.segment(0,2) - current.segment(0,2)).cast<float>().norm();

            //std::cout << "Neighbor coordinates: " << neighbor.transpose() << "   Edge Cost: " << edge_cost <<"\n";

            float new_cost = new_grid_[current[2]](current[0], current[1]).cost + edge_cost;
            if (new_cost < new_grid_[neighbor[2]](neighbor[0], neighbor[1]).cost) {
               new_grid_[neighbor[2]](neighbor[0], neighbor[1]).cost = new_cost;
               
               float heuristic = (neighbor.segment(0,2) - goal.segment(0,2)).cast<float>().norm(); // Euclidean distance
               //float heuristic = abs(dist_to_goal[0]) + abs(dist_to_goal[1]); // Manhattan distance
               pri_queue.Push(neighbor, new_cost + heuristic);
               new_grid_[neighbor[2]](neighbor[0], neighbor[1]).parent = current;
            }  
         }
      }

   }

   // Store solution (convert unordered map to vector)
   global_path_.clear();
   global_path_.push_back(gridToPoint(current.segment(0,2)));

   Eigen::Vector3i pt = current;
   
   while (new_grid_[pt[2]](pt[0], pt[1]).parent != start) {
      
      Vector2f new_point = gridToPoint(new_grid_[pt[2]](pt[0], pt[1]).parent.segment(0,2));
      Vector2f old_point = global_path_[global_path_.size()-1];
      if ((new_point - old_point).norm() > 0.15) {
         // Far point, interpolate
         global_path_.push_back(new_point*0.2 + old_point*0.8);
         global_path_.push_back(new_point*0.4 + old_point*0.6);
         global_path_.push_back(new_point*0.6 + old_point*0.4);
         global_path_.push_back(new_point*0.8 + old_point*0.2);
         global_path_.push_back(new_point);
      } else {
         // Close point, no interpolation
         global_path_.push_back(new_point);
      }
      pt = new_grid_[pt[2]](pt[0], pt[1]).parent;
   }

   std::reverse(global_path_.begin(), global_path_.end());

   this->path_index_ = 0;
}

// Updates the path index based on the current location of the vehicle
void Global_Planner::UpdatePathIndex(Vector2f veh_loc, float veh_angle) {

   if (path_index_ < (int)global_path_.size() - 1) {
      float dist_to_current_point = (global_path_[path_index_] - veh_loc).norm();
      float dist_to_next_point = (global_path_[path_index_ + 1] - veh_loc).norm();
      if (dist_to_next_point < dist_to_current_point) {
         path_index_++;
      }

   }

}

// Checks whether the global path is still valid, recomputing the path if not
void Global_Planner::CheckPathValid(Vector2f veh_loc, float veh_angle) {

   if ((global_path_[path_index_] - veh_loc).norm() > 2.) {
      ComputeGlobalPath(veh_loc, veh_angle);
   }

}

// Plots the global path plan to the provided visualization message
void Global_Planner::PlotGlobalPathVis(amrl_msgs::VisualizationMsg& vis_msg) {

   for (Eigen::Vector2f pt : global_path_) {
      visualization::DrawCross(pt, 0.05, 0x000000, vis_msg);
   }

   // Visualize map
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
   visualization::DrawCross(local_nav_goal_, .5, 0x39B81D, vis_msg);
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

std::vector<Eigen::Vector3i> Global_Planner::GetNeighborsNew(Eigen::Vector3i current) {
   vector<Eigen::Vector3i> neighbors = vector<Eigen::Vector3i>();
   
   int current_ang = current[2];

   Eigen::Vector3i pot_neighbors[8][3] = {
      {
         // 0
         Eigen::Vector3i(1, 0, 0),
         Eigen::Vector3i(10, 5, 1),
         Eigen::Vector3i(10, -5, 7)
      },
      {
         // 1
         Eigen::Vector3i(1, 1, 1),
         Eigen::Vector3i(5, 10, 2),
         Eigen::Vector3i(10, 5, 0)
      },
      {
         // 2
         Eigen::Vector3i(0, 1, 2),
         Eigen::Vector3i(-5, 10, 3),
         Eigen::Vector3i(5, 10, 1)
      },
      {
         // 3
         Eigen::Vector3i(-1, 1, 3),
         Eigen::Vector3i(-10, 5, 4),
         Eigen::Vector3i(-5, 10, 2)
      },
      {
         // 4
         Eigen::Vector3i(-1, 0, 4),
         Eigen::Vector3i(-10, -5, 5),
         Eigen::Vector3i(-10, 5, 3)
      },
      {
         // 5
         Eigen::Vector3i(-1, -1, 5),
         Eigen::Vector3i(-5, -10, 6),
         Eigen::Vector3i(-10, -5, 4)
      },
      {
         // 6
         Eigen::Vector3i(0, -1, 6),
         Eigen::Vector3i(5, -10, 7),
         Eigen::Vector3i(-5, -10, 5)
      },
      {
         // 7
         Eigen::Vector3i(1, -1, 7),
         Eigen::Vector3i(10, -5, 0),
         Eigen::Vector3i(5, -10, 6)
      }
   };
   
   // Eigen::Vector3i pot_neighbors[8][3] = {
   //    {
   //       // 0
   //       Eigen::Vector3i(1, 0, 0),
   //       Eigen::Vector3i(5, 2, 1),
   //       Eigen::Vector3i(5, -2, 7)
   //    },
   //    {
   //       // 1
   //       Eigen::Vector3i(1, 1, 1),
   //       Eigen::Vector3i(2, 5, 2),
   //       Eigen::Vector3i(5, 2, 0)
   //    },
   //    {
   //       // 2
   //       Eigen::Vector3i(0, 1, 2),
   //       Eigen::Vector3i(-2, 5, 3),
   //       Eigen::Vector3i(2, 5, 1)
   //    },
   //    {
   //       // 3
   //       Eigen::Vector3i(-1, 1, 3),
   //       Eigen::Vector3i(-5, 2, 4),
   //       Eigen::Vector3i(-2, 5, 2)
   //    },
   //    {
   //       // 4
   //       Eigen::Vector3i(-1, 0, 4),
   //       Eigen::Vector3i(-5, -2, 5),
   //       Eigen::Vector3i(-5, 2, 3)
   //    },
   //    {
   //       // 5
   //       Eigen::Vector3i(-1, -1, 5),
   //       Eigen::Vector3i(-2, -5, 6),
   //       Eigen::Vector3i(-5, -2, 4)
   //    },
   //    {
   //       // 6
   //       Eigen::Vector3i(0, -1, 6),
   //       Eigen::Vector3i(2, -5, 7),
   //       Eigen::Vector3i(-2, -5, 5)
   //    },
   //    {
   //       // 7
   //       Eigen::Vector3i(1, -1, 7),
   //       Eigen::Vector3i(5, -2, 0),
   //       Eigen::Vector3i(2, -5, 6)
   //    }
   // };

   // std::cout << "current loc is wall " << grid_(current[0], current[1]).is_wall << std::endl;

   for (Eigen::Vector3i neighbor : pot_neighbors[current_ang]){

      bool is_valid_neighbor = true;

      Eigen::Vector3i cur;
      cur[0] = current[0];
      cur[1] = current[1];

      Eigen::Vector3i nei = neighbor;
      nei[0] = neighbor[0] + cur[0];
      nei[1] = neighbor[1] + cur[1];

      float slope = (float)neighbor[1] / (float)neighbor[0];

      if (abs(slope) <= 1.) {
         int incr = (neighbor[0] >= 0) - (neighbor[0] < 0);
         for (int x = 0; abs(x) < abs(neighbor[0]); x += incr) {
            int y = (int) round(slope * (float)x);
            if (new_grid_[nei[2]](cur[0] + x, cur[1] + y).is_wall) { 
               is_valid_neighbor = false;
            }
         }
      } else {
         int incr = (neighbor[1] >= 0) - (neighbor[1] < 0);
         for (int y = 0; abs(y) < abs(neighbor[1]); y += incr) {
            int x = (int) round((float)y / slope);
            if (new_grid_[nei[2]](cur[0] + x, cur[1] + y).is_wall) { 
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

