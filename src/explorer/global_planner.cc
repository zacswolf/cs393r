

#include "global_planner.h"
#include "shared/math/line2d.h"
#include "unordered_map"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Vector2i;
using Eigen::Vector3i;
using std::vector;
using geometry::line2f;


// HARD CODED IN EVIDENCE GRID TODO: FIX
DEFINE_double(raster_pixel_size, 0.1, "Distance between pixels in the raster");

DEFINE_int32(num_angles, 8, "Number of angles to use");

// Constuctor
Global_Planner::Global_Planner(EvidenceGrid& evidence_grid): at_path_end_(false), path_algo_(PathAlgo::Mixed), evidence_grid_(evidence_grid) {
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
      std::cout << "<GP local> Error Global path is empty\n";
      return Eigen::Vector3f(0.,0.,0.);
   }

   // If at the end, move forward
   if (path_index_ == (int)global_path_.size() - 1) {
      return Eigen::Vector3f(1.,0.,0.);
   }

   UpdatePathIndex(veh_loc, veh_angle);
   bool going_backward = (global_path_[path_index_][2] > 0);

   at_path_end_ = path_index_ >= ((int)global_path_.size()-1) - 2;

   int path_lookahead = 7;

   int local_path_idx = std::min(path_index_ + path_lookahead, (int)global_path_.size()-1);

   if (local_path_idx >= 1+path_index_) {
      int p_idx = path_index_;
      bool direction = global_path_[p_idx][2] > 0;
      for (int i=p_idx; i < local_path_idx; i++) {
         bool step_dir = (global_path_[i+1][2] > 0);
         if (step_dir != direction) {
            local_path_idx = i+1;
            break;
         }
      }
   }

   local_goal = global_path_[local_path_idx];
   Vector2f local_goal_2f = local_goal.segment(0,2);
   Eigen::Rotation2Df r(-veh_angle);
   local_goal_2f = r * (local_goal_2f - veh_loc);

   // Normalize
   //local_goal_2f = local_goal_2f.array() / local_goal_2f.norm();

   // bool going_backward_lookahead = (local_goal[2] > 0);
   // // Flip direction if moving the opposite direction as our goal
   // if (going_backward != going_backward_lookahead) {
   //    std::cout << "Switching directions....\n";
   //    local_goal_2f[1] *= -1;
   // }
   
   local_goal[2] = going_backward;
   
   local_goal[0] = local_goal_2f[0];
   local_goal[1] = local_goal_2f[1];
   local_nav_goal_ = local_goal;
   return local_goal;
   
}

void Global_Planner::SimpleAStar(Vector2i veh_loc, Vector2i goal_loc) {

   SimpleQueue<Eigen::Vector2i, float> pri_queue;

   const Eigen::Vector2i start = veh_loc;
   
   pri_queue.Push(start, 0); // Push initial location

   for (int i = 0; i < simple_grid_.rows(); i++) {
      for (int j = 0; j < simple_grid_.cols(); j++) {
         simple_grid_(i, j).cost = std::numeric_limits<float>::max();
      }
   }

   simple_grid_(start[0], start[1]).cost = 0;

   // Begin A*
   std::cout << "<GP A*> Beginning Simple A*\n";
   Eigen::Vector2i current;
   bool found_path = false;
   while (!pri_queue.Empty()) {
      current = pri_queue.Pop();
      Vector2f rel_to_goal = current.cast<float>() - goal_loc.cast<float>();
      if (rel_to_goal.norm() < NUM_PIXELS_GOAL) {
         // std::cout << rel_to_goal.norm() << "\n\n";
         found_path = true;
         break;
      } else {
         std::vector<Vector2i> neighbors = GetSimpleNeighbors(current);
         for (const Eigen::Vector2i& neighbor : neighbors) {
            float edge_cost = (neighbor - current).cast<float>().norm();

            // Neighbors can be in wall padding if current is in wall padding
            // We want to penalize moving within wall padding so we add 10 map pixels to the edge cost
            bool neighbor_in_padding = (wall_grid_(neighbor[0], neighbor[1]) == 2);
            if (neighbor_in_padding) {
               edge_cost *= 5;
            }
            
            float new_cost = simple_grid_(current[0], current[1]).cost + edge_cost;
            if (new_cost < simple_grid_(neighbor[0], neighbor[1]).cost) {
               simple_grid_(neighbor[0], neighbor[1]).cost = new_cost;
               
               // Note no longer optimal with the 1.0
               float heuristic = 1.0 *(neighbor - goal_loc).cast<float>().norm(); // Euclidean distance
               
               pri_queue.Push(neighbor, new_cost + heuristic);
               simple_grid_(neighbor[0], neighbor[1]).parent = current;
            }  
         }
      }
   }

   // Error if we couldn't find a path
   // TODO: do something about this
   if (!found_path) {
      std::cout << "<GP SimpleA*> Could not find path\n";
   }
   
   // Store solution (convert unordered map to vector)
   simple_global_path_.clear();

   Eigen::Vector2f path_point = gridToPoint(current);
   simple_global_path_.push_back(path_point);

   if (current != start) { //TODO: remove

      Eigen::Vector2i loc = current;
      Eigen::Vector2i parent_loc = simple_grid_(loc[0], loc[1]).parent;

      bool done = false;
      while (!done) {
      
         parent_loc = simple_grid_(loc[0], loc[1]).parent;
         Vector2f new_point = gridToPoint(parent_loc);
         //Vector2f old_point = gridToPoint(loc);
         
         simple_global_path_.push_back(new_point);

         if (simple_grid_(loc[0], loc[1]).parent == start) {
            done = true;
         } else {
            loc = simple_grid_(loc[0], loc[1]).parent;
         }
      }
   }

   std::reverse(simple_global_path_.begin(), simple_global_path_.end());

   std::cout << "<GP A*> Finished Simple A*\n";

}

void Global_Planner::ComplexAStar(Vector2i veh_loc, float veh_angle, Vector2i goal_loc) {

   int rounded_angle_ind = ((int) round(math_util::AngleMod(veh_angle) * FLAGS_num_angles / (2 * M_PI)) );
   rounded_angle_ind = (rounded_angle_ind % NUM_ANGLES + NUM_ANGLES) % NUM_ANGLES;
   // std::cout << "Angle Ind: " << rounded_angle_ind << "\n";

   SimpleQueue<Eigen::Vector4i, float> pri_queue;

   const Eigen::Vector4i start = Eigen::Vector4i(veh_loc[0], veh_loc[1], rounded_angle_ind, 0);
   
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
      Vector2f rel_to_goal = current.cast<float>().segment(0,2) - goal_loc.cast<float>().segment(0,2);
      if (rel_to_goal.norm() < NUM_PIXELS_GOAL) {
         //std::cout << rel_to_goal.norm() << "\n";
         break;
      } else {
         vector<Eigen::Vector4i> neighbors = GetComplexNeighbors(current);
         for (Eigen::Vector4i& neighbor : neighbors) {
            bool is_backward = neighbor[3];
            float edge_cost = (neighbor.segment(0,2) - current.segment(0,2)).cast<float>().norm();

            // Penalize edge_cost if we have to go backwards
            if (is_backward) {
               edge_cost *= 12;
            }

            // Neighbors can be in wall padding if current is in wall padding
            // We want to penalize moving within wall padding so we add 10 map pixels to the edge cost
            bool neighbor_in_padding = (wall_grid_(neighbor[0], neighbor[1]) == 2);
            if (neighbor_in_padding){
               edge_cost *= 5;
            }
            
            float new_cost = grid_[current[2]](current[0], current[1]).cost + edge_cost;
            if (new_cost < grid_[neighbor[2]](neighbor[0], neighbor[1]).cost) {
               grid_[neighbor[2]](neighbor[0], neighbor[1]).cost = new_cost;
               
               // Note no longer optimal with the 1.1
               float heuristic = 1.1 *(neighbor.segment(0,2) - goal_loc.segment(0,2)).cast<float>().norm(); // Euclidean distance
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
            // step size chosen to be approximately the size of the small forward steps while also being evenly spaced / divisible
            

            float dist = (new_point.segment(0,2) - old_point.segment(0,2)).norm();
            Vector2f norm_vec = (new_point.segment(0,2) - old_point.segment(0,2)).array() / dist;
            //std::cout << "Dist: " << dist << " --- Vec: " << norm_vec.transpose() << "\n";
            //std::cout << "    Old Point: " << old_point.segment(0,2).transpose() << "\n";
            float step_size = 0.1;
            for (float step = step_size; step <= dist; step += step_size) {
               Vector2f interp_point_2f = old_point.segment(0,2) + norm_vec * step;
               //std::cout << "    Int Point: " << interp_point_2f.transpose() << "\n";
               Vector3f interp_point_3f = Vector3f(interp_point_2f[0], interp_point_2f[1], old_point[2]);
               
               global_path_.push_back(interp_point_3f);
            }
            //std::cout << "    New Point: " << new_point.segment(0,2).transpose() << "\n";
            Vector3f mod_new_point = new_point;
            mod_new_point[2] = old_point[2];
            global_path_.push_back(mod_new_point);
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
}

// Computes the global path from the vehicle's pose to the global nav goal
void Global_Planner::ComputeGlobalPath(Vector2f veh_point, float veh_angle) {

   Eigen::Vector2i goal_loc = pointToGrid(global_nav_goal_);
   Eigen::Vector2i veh_loc = pointToGrid(veh_point);
   
   this->path_index_ = 0;

   if (path_algo_ == PathAlgo::Mixed) {
      // Compute simple path
      SimpleAStar(veh_loc, goal_loc);

      // Compute lattice planned A* path
      // std::cout << simple_global_path_.size() << "\n";
      int look_ahead = std::min(mixed_lookahead, (int)simple_global_path_.size() - 1);
      Vector2i simple_goal_loc = pointToGrid(simple_global_path_[look_ahead]);
      ComplexAStar(veh_loc, veh_angle, simple_goal_loc);

      // Remove beginning of simple path
      simple_global_path_ = std::vector<Vector2f>(simple_global_path_.begin() + look_ahead - 1, simple_global_path_.end());

   } else if (path_algo_ == PathAlgo::Complex) {
      // Compute lattice planned A* path
      ComplexAStar(veh_loc, veh_angle, goal_loc); // old
   } else if (path_algo_ == PathAlgo::Simple) {
      SimpleAStar(veh_loc, goal_loc);
      global_path_.clear();
      for (Vector2f& pt : simple_global_path_) {
         global_path_.push_back(Vector3f(pt[0], pt[1], 0));
      }
   } else {
      std::cout << "<GP> Compute Global Path has invalid Path Algo\n";
   }

   at_path_end_ = false;

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

   // Plot simple path
   for (Eigen::Vector2f pt : simple_global_path_) {
      visualization::DrawCross(pt, 0.05, 0xAAAAAA, vis_msg);
   }

   // Plot complex path
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

std::vector<Eigen::Vector4i> Global_Planner::GetComplexNeighbors(Eigen::Vector4i current) {
   vector<Eigen::Vector4i> neighbors = vector<Eigen::Vector4i>();
   
   int current_ang = current[2];

   // (x, y, angle, is_backward)
   const static Eigen::Vector4i pot_offsets[8][4] = {
      {
         // 0
         Eigen::Vector4i(1, 0, 0, 0),
         Eigen::Vector4i(10, 5, 1, 0),
         Eigen::Vector4i(10, -5, 7, 0),
         Eigen::Vector4i(-4, 0, 0, 1),
         // Eigen::Vector4i(-10, 5, 7, 1),
         // Eigen::Vector4i(-10, -5, 1, 1),
      },
      {
         // 1
         Eigen::Vector4i(1, 1, 1, 0),
         Eigen::Vector4i(5, 10, 2, 0),
         Eigen::Vector4i(10, 5, 0, 0),
         Eigen::Vector4i(-4, -4, 1, 1),
         // Eigen::Vector4i(-10, -5, 0, 1),
         // Eigen::Vector4i(-5, -10, 2, 1)
      },
      {
         // 2
         Eigen::Vector4i(0, 1, 2, 0),
         Eigen::Vector4i(-5, 10, 3, 0),
         Eigen::Vector4i(5, 10, 1, 0),
         Eigen::Vector4i(0, -4, 2, 1),
         // Eigen::Vector4i(-5, -10, 1, 1),
         // Eigen::Vector4i(5, -10, 3, 1)
      },
      {
         // 3
         Eigen::Vector4i(-1, 1, 3, 0),
         Eigen::Vector4i(-10, 5, 4, 0),
         Eigen::Vector4i(-5, 10, 2, 0),
         Eigen::Vector4i(4, -4, 3, 1),
         // Eigen::Vector4i(5, -10, 2, 1),
         // Eigen::Vector4i(10, -5, 4, 1)
      },
      {
         // 4
         Eigen::Vector4i(-1, 0, 4, 0),
         Eigen::Vector4i(-10, -5, 5, 0),
         Eigen::Vector4i(-10, 5, 3, 0),
         Eigen::Vector4i(4, 0, 4, 1),
         // Eigen::Vector4i(10, -5, 3, 1),
         // Eigen::Vector4i(10, 5, 5, 1)
      },
      {
         // 5
         Eigen::Vector4i(-1, -1, 5, 0),
         Eigen::Vector4i(-5, -10, 6, 0),
         Eigen::Vector4i(-10, -5, 4, 0),
         Eigen::Vector4i(4, 4, 5, 1),
         // Eigen::Vector4i(10, 5, 4, 1),
         // Eigen::Vector4i(5, 10, 6, 1)
      },
      {
         // 6
         Eigen::Vector4i(0, -1, 6, 0),
         Eigen::Vector4i(5, -10, 7, 0),
         Eigen::Vector4i(-5, -10, 5, 0),
         Eigen::Vector4i(0, 4, 6, 1),
         // Eigen::Vector4i(5, 10, 5, 1),
         // Eigen::Vector4i(-5, 10, 7, 1)
      },
      {
         // 7
         Eigen::Vector4i(1, -1, 7, 0),
         Eigen::Vector4i(10, -5, 0, 0),
         Eigen::Vector4i(5, -10, 6, 0),
         Eigen::Vector4i(-4, 4, 7, 1),
         // Eigen::Vector4i(-5, 10, 6, 1),
         // Eigen::Vector4i(-10, 5, 0, 1)
      }
   };

   // std::cout << "current loc is wall " << wall_grid_(current[0], current[1]) << std::endl;

   bool in_padding = (wall_grid_(current[0], current[1]) == 2);

   for (const Eigen::Vector4i& offset : pot_offsets[current_ang]){

      bool is_valid_neighbor = true;

      Eigen::Vector4i pot_neighbor = offset;
      pot_neighbor[0] += current[0];
      pot_neighbor[1] += current[1];

      float slope = (float)offset[1] / (float)offset[0];

      if (abs(slope) <= 1.) {
         int incr = (offset[0] >= 0) - (offset[0] < 0);
         
         // Interp from current to pot_neighbor
         for (int x = 0; abs(x) < abs(offset[0]); x += incr) {
            int y = (int) round(slope * (float)x);

            Vector2i interp = current.segment(0,2) + Vector2i(x,y);
            bool is_known = evidence_grid_.is_known(interp);
            bool is_wall = (wall_grid_(interp[0], interp[1]) == 1) || ((wall_grid_(interp[0], interp[1]) == 2) && !in_padding);

            is_valid_neighbor &= (is_known && !is_wall);

            // if ((wall_grid_(interp) == 1) || ((wall_grid_(interp) == 2) && !in_padding)) { 
            //    is_valid_neighbor = false;
            // }
         }
      } else {
         int incr = (offset[1] >= 0) - (offset[1] < 0);

         // Interp from current to pot_neighbor
         for (int y = 0; abs(y) < abs(offset[1]); y += incr) {
            int x = (int) round((float)y / slope);

            Vector2i interp = current.segment(0,2) + Vector2i(x,y);
            bool is_known = evidence_grid_.is_known(interp);
            bool is_wall = (wall_grid_(interp[0], interp[1]) == 1) || ((wall_grid_(interp[0], interp[1]) == 2) && !in_padding);

            is_valid_neighbor &= (is_known && !is_wall);
         }
      }

      if (is_valid_neighbor) {
         neighbors.push_back(pot_neighbor);
      }
   }

   return neighbors;
}

std::vector<Eigen::Vector2i> Global_Planner::GetSimpleNeighbors(Eigen::Vector2i current) {
   vector<Eigen::Vector2i> neighbors = vector<Eigen::Vector2i>();

   // (x, y)
   static const Eigen::Vector2i pot_offsets[] = {
      Vector2i(-1,0),
      Vector2i(1,0),
      Vector2i(0,-1),
      Vector2i(0,1),
      Vector2i(-1,-1),
      Vector2i(-1,1),
      Vector2i(1,-1),
      Vector2i(1,1)
   };

   bool in_padding = (wall_grid_(current[0], current[1]) == 2);

   for (const Eigen::Vector2i& offset : pot_offsets) {
      Vector2i pot_neighbor = current + offset;
      bool is_known = evidence_grid_.is_known(pot_neighbor);
      bool is_wall = wall_grid_(pot_neighbor[0], pot_neighbor[1]) == 1 || ((wall_grid_(pot_neighbor[0], pot_neighbor[1]) == 2) && !in_padding);

      if (is_known && !is_wall) {
         neighbors.push_back(pot_neighbor);
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

void Global_Planner::updateWallGrid(std::unordered_set<Eigen::Vector2i, matrix_hash<Eigen::Vector2i>> new_walls, bool soft_update) {
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
