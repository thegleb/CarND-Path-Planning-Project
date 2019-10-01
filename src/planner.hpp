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
#include <stdio.h>
#include <math.h>
#include <stdio.h>

#include "helpers.h"
#include "spline.h"
#include "vehicle.hpp"
#include "prediction.hpp"

using std::vector;
using std::map;

class Planner {
public:
  // Constructors
  Planner();
  virtual ~Planner();
  
  void predict_ego(string state, Prediction &candidate, map<int, Prediction> &predictions);
  void predict_other_cars(map<int, vector<double>> sensor_fusion, map<int, Prediction> &predictions, int num_steps);

  vector<vector<double>> generate_trajectory_for_lane(int lane, int new_lane, double current_speed);
  vector<vector<double>> generate_trajectory_for_lane_change(string state);

  void prediction_for_lane_keep(Prediction &prediction, string state, double s, map<int, Prediction> &predictions, int lane, double current_v);
  void prediction_for_lane_change(Prediction &prediction, string state, double s, map<int, Prediction> &predictions);
  
  void update_position(double car_x, double car_y, double car_s, double car_d, double car_yaw);
  void update_prev_position(vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d);

  bool get_vehicle_behind(map<int, Prediction> &predictions, int lane, Prediction &vehicle_behind);
  bool get_vehicle_ahead(map<int, Prediction> &predictions, int lane, Prediction &vehicle_ahead);

  Prediction choose_next_state();
  double adjust_speed(double current_v, double target_v);

  map<int, vector<double>> sensor_fusion;
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  Vehicle ego;
};

#endif /* planner_hpp */
