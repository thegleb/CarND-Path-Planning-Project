//
//  planner.hpp
//  CarND-Path-Planning-Project
//
//  Created by Gleb on 9/24/19.
//  Copyright © 2019 Gleb. All rights reserved.
//

#ifndef planner_hpp
#define planner_hpp

#include <vector>
#include <map>
#include <cmath>
#include <cstdio>

#include "helpers.h"
#include "spline.h"
#include "vehicle.hpp"
#include "plan.hpp"

using std::vector;
using std::map;

class Planner {
public:
  // Constructors
  Planner();
  virtual ~Planner();
  
  void predict_ego(string state, Plan &candidate, map<int, Plan> &predictions);
  static void predict_other_cars(map<int, vector<double>> sensor_fusion, map<int, Plan> &predictions, int num_steps);

  vector<vector<double>> generate_trajectory_for_lane(int lane, int new_lane, double current_speed);

  void prediction_for_lane_keep(Plan &prediction, string state, double s, map<int, Plan> &predictions, int lane, double current_v);
  void prediction_for_lane_change(Plan &prediction, string state, double s, map<int, Plan> &predictions);
  
  void update_position(double car_x, double car_y, double car_s, double car_d, double car_yaw);
  void update_prev_position(vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d);

  bool get_vehicle_behind(map<int, Plan> &predictions, int lane, Plan &vehicle_behind);
  bool get_vehicle_ahead(map<int, Plan> &predictions, int lane, Plan &vehicle_ahead);

  Plan choose_next_plan();
  static double get_updated_speed(double current_v, double target_v);

  map<int, vector<double>> sensor_fusion;
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  Vehicle ego;
};

#endif /* planner_hpp */
