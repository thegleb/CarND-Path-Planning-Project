//
//  planner.cpp
//  CarND-Path-Planning-Project
//
//  Created by Gleb on 9/24/19.
//  Copyright © 2019 Gleb. All rights reserved.
//

#include "planner.hpp"

Planner::Planner() {}
Planner::~Planner() {}

#define MIN_DETECTED_DISTANCE 50
#define MIN_FOLLOWING_DISTANCE 40

float lane_speed(Prediction candidate, const map<int, Prediction> &predictions, Vehicle &ego) {
  // find nearest vehicle in that lane that would be an obstacle
  float min_speed = ego.speed;
  float min_distance = MIN_FOLLOWING_DISTANCE;
  for (map<int, Prediction>::const_iterator it = predictions.begin();
       it != predictions.end(); ++it) {
    int key = it->first;
    Prediction prediction = it->second;
    if (prediction.lane == candidate.lane &&
        key != -1 &&
        prediction.v < min_speed &&
        prediction.s > candidate.s &&
        prediction.s - candidate.s < min_distance) {
      min_distance = prediction.s - candidate.s;
      min_speed = prediction.v;
    }
  }

//  printf("intended lane:%i speed:%f\n", candidate.lane, min_speed);
  return min_speed;
}

float choose_fast_lane(const Prediction &candidate,
                        const map<int, Prediction> &predictions,
                      Vehicle &ego) {

  // if target lane is slower than max speed
  float intended_lane_speed = lane_speed(candidate, predictions, ego);
  if (ego.speed < intended_lane_speed) {
    return 0;
  }

  return (ego.speed - intended_lane_speed) / ego.speed;
}

float maximize_speed(const Prediction &candidate, const map<int, Prediction> &predictions, Vehicle &ego) {
  float diff = TARGET_V - candidate.v;
  return diff / TARGET_V;
}

float valid_lane(const Prediction &candidate, const map<int, Prediction> &predictions, Vehicle &ego) {
  return candidate.lane >= 0 && candidate.lane <= 2 ? 0 : 1;
}

float dont_merge_when_car_nearby(const Prediction &candidate, const map<int, Prediction> &predictions, Vehicle &ego) {
  float min_distance = 30;
  if (candidate.state.compare("KL") != 0) {
    for (map<int, Prediction>::const_iterator it = predictions.begin();
         it != predictions.end(); ++it) {
      int key = it->first;
      Prediction prediction = it->second;
      if (prediction.lane == candidate.lane &&
          key != -1 &&
          abs(prediction.s - candidate.s) < min_distance) {
        return 1 - abs(prediction.s - candidate.s) / min_distance;
      }
    }
  }

  return 0;
}

float dont_cut_off_faster_cars(const Prediction &candidate, const map<int, Prediction> &predictions, Vehicle &ego) {
  float max_speed_behind = ego.speed;
  float min_distance = MIN_DETECTED_DISTANCE;
  if (candidate.state.compare("KL") != 0) {
    for (map<int, Prediction>::const_iterator it = predictions.begin();
         it != predictions.end(); ++it) {
      int key = it->first;
      Prediction prediction = it->second;
      if (prediction.lane == candidate.lane &&
          key != -1 &&
          prediction.v > max_speed_behind &&
          prediction.s < candidate.s &&
          candidate.s - prediction.s < min_distance) {
        min_distance = candidate.s - prediction.s;
        max_speed_behind = prediction.v;
      }
    }
  }
  return max_speed_behind > (ego.speed + 5) ? (max_speed_behind - ego.speed) : 0;
}

//
float calculate_cost(const Prediction &candidate, const map<int, Prediction> &predictions, Vehicle &ego) {
  // Sum weighted cost functions to get total cost for trajectory.
//  map<string, float> trajectory_data = get_helper_data(trajectory,
//                                                       predictions);
  float cost = 0.0;
  
//   TODO: Add additional cost functions here.
  vector<std::function<float(const Prediction &,
                             const map<int, Prediction> &,
                             Vehicle &ego)>> cf_list = {
    maximize_speed,
    choose_fast_lane,
    valid_lane,
    dont_cut_off_faster_cars,
    dont_merge_when_car_nearby
  };
  vector<float> weight_list = {1000, 3000, 1000000, 10000, 100000};
  
  printf("%s:\n", candidate.state.c_str());
  printf("weight: maximize_speed: %f\n", cf_list[0](candidate, predictions, ego));
  printf("weight: choose_fast_lane: %f\n", cf_list[1](candidate, predictions, ego));
  printf("weight: valid_lane: %f\n", cf_list[2](candidate, predictions, ego));
  printf("weight: dont_cut_off_faster_cars: %f\n", cf_list[3](candidate, predictions, ego));
  printf("weight: dont_merge_when_car_nearby: %f\n", cf_list[4](candidate, predictions, ego));
  printf("\n");

  for (int i = 0; i < cf_list.size(); ++i) {
    cost += weight_list[i] * cf_list[i](candidate, predictions, ego);
  }
  
  return cost;
}

Prediction Planner::choose_next_state() {
  /**
   * Here you can implement the transition_function code from the Behavior
   *   Planning Pseudocode classroom concept.
   *
   * @param A predictions map. This is a map of vehicle id keys with predicted
   *   vehicle trajectories as values. Trajectories are a vector of Vehicle
   *   objects representing the vehicle at the current timestep and one timestep
   *   in the future.
   * @output The best (lowest cost) trajectory corresponding to the next ego
   *   vehicle state.
   *
   * Functions that will be useful:
   * 1. successor_states - Uses the current state to return a vector of possible
   *    successor states for the finite state machine.
   * 2. generate_trajectory - Returns a vector of Vehicle objects representing
   *    a vehicle trajectory, given a state and predictions. Note that
   *    trajectory vectors might have size 0 if no possible trajectory exists
   *    for the state.
   * 3. calculate_cost - Computes the cost for a trajectory.
   *
   */
  // generate predictions 30 steps ahead... very estimated

  vector<float> costs;
  map<int, Prediction> predictions;
  Prediction candidate;
  vector<Prediction> candidates;

  ego.speed = adjust_speed(ego.speed, ego.target_speed);
  predict_other_cars(sensor_fusion, predictions, 70);

  vector<string> states = ego.next_states();
  for (int i = 0; i < states.size(); i++) {
    string state = states[i];
    predict_ego(state, candidate, predictions);
    float cost = calculate_cost(candidate, predictions, ego);
    costs.push_back(cost);
    candidates.push_back(candidate);
  }

  float min_cost = 10000000000;
  int min_cost_idx = 0;
  for (int i = 0; i < costs.size(); i++) {
    printf("%s: %f\n", candidates[i].state.c_str(), costs[i]);
    if (costs[i] < min_cost) {
      min_cost = costs[i];
      min_cost_idx = i;
    }
  }
  printf("\n");

  Prediction lowest_cost = candidates[min_cost_idx];
  // update state
  ego.state = lowest_cost.state;
  ego.target_lane = get_lane_from_d(lowest_cost.d);
  ego.target_speed = lowest_cost.v;
//  printf("state:%s v:%f\n", lowest_cost.state.c_str(), lowest_cost.v);

  return lowest_cost;
}


vector<vector<double>> Planner::generate_trajectory_for_lane(int lane, int new_lane, double current_v) {

  int tween = new_lane - lane;
  // get the next 3 waypoints that we'd like to hit:
  // 30 meters, 60 meters, and 90 meters away
  vector<vector<double>> target_waypoints;
  int increment = current_v > 30 ? current_v * 1.5 : 40;
  for (int i = increment; i <= increment * 3; i+= increment) {
    target_waypoints.push_back(getXY(ego.s + i, (2 + 4 * (lane + i / (increment * 3) * tween)), map_waypoints_s, map_waypoints_x, map_waypoints_y));
  }
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  vector<double> pts_x;
  vector<double> pts_y;
  double ref_x = ego.x;
  double ref_y = ego.y;
  double ref_yaw = deg2rad(ego.yaw);
  int prev_size = ego.previous_path_x.size();

  // make sure we have at least a couple points to start our trajectory generation
  if (prev_size < 2) {
    // initialize by extrapolating a previous position
    // if we don't have enough previous points yet
    double prev_car_x = ego.x - cos(ego.yaw);
    double prev_car_y = ego.y - sin(ego.yaw);

    pts_x.push_back(prev_car_x);
    pts_x.push_back(ego.x);

    pts_y.push_back(prev_car_y);
    pts_y.push_back(ego.y);
  } else {
    // we use the last 2 points as the basis for generating the next bunch
    ref_x = ego.previous_path_x[prev_size - 1];
    ref_y = ego.previous_path_y[prev_size - 1];

    double ref_x_prev = ego.previous_path_x[prev_size - 2];
    double ref_y_prev = ego.previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
    
    pts_x.push_back(ref_x_prev);
    pts_x.push_back(ref_x);
    
    pts_y.push_back(ref_y_prev);
    pts_y.push_back(ref_y);
  }
  
  for (int i = 0; i < ego.previous_path_x.size(); i++) {
    next_x_vals.push_back(ego.previous_path_x[i]);
    next_y_vals.push_back(ego.previous_path_y[i]);
  }

  // now add the target waypoints to interpolate
  for (int i = 0; i < target_waypoints.size(); i++) {
    pts_x.push_back(target_waypoints[i][0]);
    pts_y.push_back(target_waypoints[i][1]);
  }

  // convert points to car coordinates
  for (int i = 0; i < pts_x.size(); i++) {
    double shift_x = pts_x[i] - ref_x;
    double shift_y = pts_y[i] - ref_y;
    
    pts_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    pts_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  // now use the points we gathered to generate a spline
  tk::spline spline_fn;
  spline_fn.set_points(pts_x, pts_y);

  // trigonometry to figure out a decently smooth trajectory
  // in this case "x" represents some marker 30 meters in front of the car
  double target_x = 30.0;
  double target_y = spline_fn(target_x);
  double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

  double x_add_on = 0;

//  float a_frac = a * D_T;
  for (int i = 1; i < MAX_PATH_POINTS - ego.previous_path_x.size(); i++) {
    double N = (target_dist / (D_T * mps_from_mph(current_v)));
    double x_point = x_add_on + target_x / N;
    double y_point = spline_fn(x_point);
    x_add_on = x_point;
    
    double x_ref = x_point;
    double y_ref = y_point;
    
    // convert back to global coordinates
    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    // offsets to the previous point(?)(todo: fix comment)
    x_point += ref_x;
    y_point += ref_y;
    
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
  return {next_x_vals, next_y_vals, {target_dist, current_v}};
}

vector<vector<double>> Planner::generate_trajectory_for_lane_change(string state) {
  if (state.compare("LCL")) {
    return generate_trajectory_for_lane(ego.lane, ego.lane + 1, ego.speed);
  } else if (state.compare("LCR")) {
    return generate_trajectory_for_lane(ego.lane, ego.lane - 1, ego.speed);
  }
}

void Planner::update_position(double car_x,
                              double car_y,
                              double car_s,
                              double car_d,
                              double car_yaw) {
  ego.x = car_x;
  ego.y = car_y;
  ego.s = car_s;
  ego.lane = get_lane_from_d(car_d);
  ego.d = car_d;
  ego.yaw = car_yaw;
//  ego.speed = car_speed;
}

void Planner::update_prev_position(vector<double> new_previous_path_x,
                                   vector<double> new_previous_path_y,
                                   double new_end_path_s,
                                   double new_end_path_d) {
  ego.previous_path_x = new_previous_path_x;
  ego.previous_path_y = new_previous_path_y;
  ego.end_path_s = new_end_path_s;
  ego.end_path_d = new_end_path_d;
}

double Planner::adjust_speed(double current_v, double target_v) {
  // If the current speed is too different from the target speed, then accelerate/decelerate to match
//  printf("target: %f\n", target_v);
  float delta = (target_v - current_v);
  if (abs(delta) > 0) {
    return current_v + mph_from_mps(MAX_A * (delta > 0 ? 1 : -1)) * D_T;
  } else {
    return current_v;
  }
}

void Planner::predict_other_cars(map<int, vector<double>> sensor_fusion, map<int, Prediction> &predictions, int num_steps) {
  // Generates predictions for non-ego vehicles to be used in trajectory generation for the ego vehicle.
  // Assumes constant velocity for other cars.

  // sensor_fusion is a map of id to {x, y, v_x, v_y, s, d}

  for (map<int, vector<double>>::iterator it = sensor_fusion.begin();
       it != sensor_fusion.end(); ++it) {
    vector<double> measurement = it->second;
    float s = measurement[4];
    float d = measurement[5];
    float v_x = measurement[2];
    float v_y = measurement[3];
    
    float v = sqrt(pow(v_x, 2) + pow(v_y, 2));
    float next_s = position_at(s, v, 0, num_steps * D_T);
    // todo: implement to account for accelerationa as well as d that changes
//      float next_v = 0;
//      if (j < num_steps - 1) {
//        next_v = position_at(i + 1) - s;
//      }
      
    predictions.insert({it->first, Prediction(next_s, d, mph_from_mps(v), "KL")});
  }
}

void Planner::prediction_for_lane_keep(Prediction &prediction,
                                       string state,
                                       double s,
                                       map<int, Prediction> &predictions,
                                       int lane,
                                       double current_v) {
  double target_v = TARGET_V;
  Prediction vehicle_ahead;
  if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
    // Keep speed of current lane so as not to collide with car in front
    target_v = fmin(vehicle_ahead.v, target_v) - 1;
  }
  
  vector<vector<double>> trajectory = generate_trajectory_for_lane(lane, lane, current_v);
  prediction.trajectory = trajectory;
  prediction.v = target_v;
  prediction.d = lane * 4 + 2;
  prediction.lane = lane;
  prediction.s = s + trajectory[2][0];
  prediction.state = state;
}

void Planner::prediction_for_lane_change(Prediction &prediction,
                                         string state,
                                         double s,
                                         map<int, Prediction> &predictions) {
  int new_lane = ego.lane;
  if (state.compare("LCL")) {
    new_lane += 1;
  } else if (state.compare("LCR")) {
    new_lane -= 1;
  }

  Prediction vehicle_behind;
  Prediction vehicle_ahead;
  double target_v = TARGET_V;
  bool has_vehicle_ahead = false;
  bool has_vehicle_behind = false;

  has_vehicle_ahead = get_vehicle_ahead(predictions, new_lane, vehicle_ahead);
  has_vehicle_behind = get_vehicle_behind(predictions, new_lane, vehicle_behind);

  if (has_vehicle_ahead && !has_vehicle_behind) {
    target_v = fmin(vehicle_ahead.v, ego.speed);
//  } else if (has_vehicle_behind && !has_vehicle_ahead) {
//    target_v = fmax(vehicle_behind.v, ego.speed);
  } else if (has_vehicle_behind && has_vehicle_ahead) {
    target_v = fmin(vehicle_behind.v, vehicle_ahead.v);
  }

  prediction.trajectory = generate_trajectory_for_lane(ego.lane, new_lane, ego.speed);
  prediction.v = target_v;
  prediction.d = new_lane * 4 + 2;
  prediction.lane = new_lane;
  prediction.s = s + prediction.trajectory[2][0];
  prediction.state = state;
}

void Planner::predict_ego(string state, Prediction &candidate, map<int, Prediction> &predictions) {
  // Given a possible next state, generate the appropriate trajectory to realize the next state.
//  printf("eval:%s, lane:%i v:%f\n", state.c_str(), ego.lane, ego.speed);
  if (state.compare("KL") == 0) {
    prediction_for_lane_keep(candidate, state, ego.s, predictions, ego.lane, ego.speed);
  } else if (state.compare("LCIP") == 0) {
    // this is the "lane change in progress" state
    prediction_for_lane_keep(candidate, state, ego.s, predictions, ego.target_lane, ego.speed);
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    prediction_for_lane_change(candidate, state, ego.s, predictions);
  }
}


bool Planner::get_vehicle_behind(map<int, Prediction> &predictions, int lane, Prediction &vehicle_behind) {
  // Returns true if a faster vehicle is found behind the current vehicle
  int min_distance = MIN_DETECTED_DISTANCE;
  bool found_vehicle = false;
  Prediction temp_vehicle;
  for (map<int, Prediction>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
    temp_vehicle = it->second;
    if (temp_vehicle.lane == ego.lane && temp_vehicle.s < ego.s && (ego.s - temp_vehicle.s) < min_distance && temp_vehicle.v > ego.speed) {
      min_distance = temp_vehicle.s;
      vehicle_behind = temp_vehicle;
      found_vehicle = true;
    }
  }
  
  return found_vehicle;
}

bool Planner::get_vehicle_ahead(map<int, Prediction> &predictions, int lane, Prediction &vehicle_ahead) {
  // Returns true if a slower vehicle is found ahead of the current vehicle
  int min_distance = MIN_DETECTED_DISTANCE;
  bool found_vehicle = false;
  Prediction temp_vehicle;
  for (map<int, Prediction>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
    temp_vehicle = it->second;
    if (temp_vehicle.lane == ego.lane &&
        temp_vehicle.s > ego.s &&
        (temp_vehicle.s - ego.s) < min_distance + 25
        ) {
//      printf("vehicle ahead:%f speed: %f\n", temp_vehicle.s - ego.s, temp_vehicle.v);

//      if (temp_vehicle.v < ego.speed) {
      min_distance = temp_vehicle.s - ego.s;
      vehicle_ahead = temp_vehicle;
      found_vehicle = true;
//      }
    }
  }
  
  return found_vehicle;
}
