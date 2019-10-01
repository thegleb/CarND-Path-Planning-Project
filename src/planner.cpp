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

#define MIN_DETECTED_DISTANCE 50.0f
#define MIN_FOLLOWING_DISTANCE 40.0f

// Calculate the lane speed for a plan
float lane_speed(Plan candidate, const map<int, Plan> &predictions, Vehicle &ego) {
  // Find nearest vehicle in that lane that would be an obstacle
  float min_speed = ego.v;
  float min_distance = MIN_FOLLOWING_DISTANCE;
  for (map<int, Plan>::const_iterator it = predictions.begin();
       it != predictions.end(); ++it) {
    int key = it->first;
    Plan prediction = it->second;
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

// Calculates a cost for choosing a lane slower than current
float choose_fast_lane(const Plan &candidate,
                        const map<int, Plan> &predictions,
                      Vehicle &ego) {

  // if target lane is slower than max speed
  float intended_lane_speed = lane_speed(candidate, predictions, ego);
  if (ego.v < intended_lane_speed) {
    return 0;
  }

  return (ego.v - intended_lane_speed) / ego.v;
}

// Calculates a cost for maximizing the velocity relative to the target velocity
float maximize_speed(const Plan &candidate, const map<int, Plan> &predictions, Vehicle &ego) {
  float diff = TARGET_V - candidate.v;
  return diff / TARGET_V;
}

// Calculates a cost of selecting an invalid lane
float valid_lane(const Plan &candidate, const map<int, Plan> &predictions, Vehicle &ego) {
  return candidate.lane >= 0 && candidate.lane <= 2 ? 0 : 1;
}

// Calculates a cost for merging when there's a car nearby
float dont_merge_when_car_nearby(const Plan &candidate, const map<int, Plan> &predictions, Vehicle &ego) {
  float min_distance = 30;
  if (candidate.state != "KL") {
    for (map<int, Plan>::const_iterator it = predictions.begin();
         it != predictions.end(); ++it) {
      int key = it->first;
      Plan prediction = it->second;
      if (prediction.lane == candidate.lane &&
          key != -1 &&
          abs(prediction.s - candidate.s) < min_distance) {
        return 1 - abs(prediction.s - candidate.s) / min_distance;
      }
    }
  }

  return 0;
}

// Calculates a cost for cutting off a fast-approaching car from behind when changing lanes
float dont_cut_off_faster_cars(const Plan &candidate, const map<int, Plan> &predictions, Vehicle &ego) {
  float max_speed_behind = ego.v;
  float min_distance = MIN_DETECTED_DISTANCE;
  if (candidate.state != "KL") {
    for (map<int, Plan>::const_iterator it = predictions.begin();
         it != predictions.end(); ++it) {
      int key = it->first;
      Plan prediction = it->second;
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
  return max_speed_behind > (ego.v + 5) ? (max_speed_behind - ego.v) : 0;
}

// Sum weighted cost functions to get total cost for plan.
float calculate_cost(const Plan &candidate, const map<int, Plan> &predictions, Vehicle &ego) {
  float cost = 0.0;

  vector<std::function<float(const Plan &,
                             const map<int, Plan> &,
                             Vehicle &ego)>> cf_list = {
    maximize_speed,
    choose_fast_lane,
    valid_lane,
    dont_cut_off_faster_cars,
    dont_merge_when_car_nearby
  };
  vector<float> weight_list = {1000, 3000, 1000000, 10000, 100000};

//  printf("%s:\n", candidate.state.c_str());
//  printf("weight: maximize_speed: %f\n", cf_list[0](candidate, predictions, ego));
//  printf("weight: choose_fast_lane: %f\n", cf_list[1](candidate, predictions, ego));
//  printf("weight: valid_lane: %f\n", cf_list[2](candidate, predictions, ego));
//  printf("weight: dont_cut_off_faster_cars: %f\n", cf_list[3](candidate, predictions, ego));
//  printf("weight: dont_merge_when_car_nearby: %f\n", cf_list[4](candidate, predictions, ego));
//  printf("\n");

  for (int i = 0; i < cf_list.size(); ++i) {
    cost += weight_list[i] * cf_list[i](candidate, predictions, ego);
  }

  return cost;
}

// Returns the best (lowest cost) plan corresponding to the next ego vehicle state
Plan Planner::choose_next_plan() {

  vector<float> costs;
  map<int, Plan> predictions;
  Plan candidate;
  vector<Plan> candidates;

  // Update car speed based on the target speed and current speed
  ego.v = get_updated_speed(ego.v, ego.target_v);

  // We predict the locations of cars 70 time steps into the future.
  // This specific number was chosen because we generate trajectories roughly 30 meters in front,
  // which corresponds to a little under 1.5 seconds at the top speed of ~50 mph (~22 m/sec).
  // 1.5 seconds -> ~70 time steps into the future assuming each time step is ~0.02 seconds.
  // Seems to work ok overall.
  predict_other_cars(sensor_fusion, predictions, 70);

  // 1. Determine what possible states the car can have given its current state
  vector<string> states = ego.next_states();

  // 2. Iterate over the states
  for (int i = 0; i < states.size(); i++) {
    string state = states[i];
    // 3. Predict position of the ego car given chosen state
    predict_ego(state, candidate, predictions);
    // 4. Calculate a cost for this chosen state based on some cost functions
    float cost = calculate_cost(candidate, predictions, ego);
    costs.push_back(cost);
    candidates.push_back(candidate);
  }

  // 5. Find minimum cost prediction
  float min_cost = 10000000000.0f;
  int min_cost_idx = 0;
  for (int i = 0; i < costs.size(); i++) {
//    printf("%s: %f\n", candidates[i].state.c_str(), costs[i]);
    if (costs[i] < min_cost) {
      min_cost = costs[i];
      min_cost_idx = i;
    }
  }
//  printf("\n");

  // 6. Using the lowest-cost plan, update ego car state and target lane.
  //    Target lane is used for intermediate "lane change in progress" (LCIP) states
  //    in order to make sure we keep track of where the car is trying to go.
  Plan lowest_cost = candidates[min_cost_idx];
  ego.state = lowest_cost.state;
  ego.target_lane = get_lane_from_d(lowest_cost.d);
  ego.target_v = lowest_cost.v;

  //  printf("state:%s v:%f\n", lowest_cost.state.c_str(), lowest_cost.v);

  return lowest_cost;
}

// Generates a trajectory given the current and target lanes.
vector<vector<double>> Planner::generate_trajectory_for_lane(int current_lane, int target_lane, double current_v) {
  int delta_lane = target_lane - current_lane;

  // 1. Get the next 3 waypoints we want to hit. These points will be used for generating the spline
  //    curve for smooth lane changes.
  vector<vector<double>> target_waypoints;

  // To minimize jerk, through a bunch of experimentation (I did not feel like doing the formal math for it)
  // I determined that picking 3 points 40, 80, and 120 meters away yields smooth lane changes, but only at
  // lower speeds.
  // Thus here I am ensuring that at minimum we use the same 30, 60, 90, 120, 150 offsets
  // for slower speeds (below 30 mph) and scale it up for higher speeds to maintain smoothness
  int increment = (int)round(current_v > 30 ? current_v * 1.25 : 30);
  for (int i = increment; i <= increment * 5; i += increment) {
    target_waypoints.push_back(getXY(
            ego.s + (float)i,
            (2 + 4 * ((float)current_lane + (float)i / ((float)increment * 5) * (float)delta_lane)),
            map_waypoints_s,
            map_waypoints_x,
            map_waypoints_y));
  }
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  vector<double> pts_x;
  vector<double> pts_y;
  double ref_x = ego.x;
  double ref_y = ego.y;
  double ref_yaw = deg2rad(ego.yaw);
  int prev_size = ego.previous_path_x.size();

  // Make sure we have at least a couple points to start our trajectory generation
  if (prev_size < 2) {
    // If we don't have enough previous points yet, initialize by extrapolating a previous position
    double prev_car_x = ego.x - cos(ego.yaw);
    double prev_car_y = ego.y - sin(ego.yaw);

    pts_x.push_back(prev_car_x);
    pts_x.push_back(ego.x);

    pts_y.push_back(prev_car_y);
    pts_y.push_back(ego.y);
  } else {
    // We use the last 2 points as the basis for generating the next bunch
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

  // Now add the target waypoints to interpolate using splines
  for (int i = 0; i < target_waypoints.size(); i++) {
    pts_x.push_back(target_waypoints[i][0]);
    pts_y.push_back(target_waypoints[i][1]);
  }

  // Convert points to car coordinates
  for (int i = 0; i < pts_x.size(); i++) {
    double shift_x = pts_x[i] - ref_x;
    double shift_y = pts_y[i] - ref_y;

    pts_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    pts_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  // Now use the points we gathered to generate a spline
  tk::spline spline_fn;
  spline_fn.set_points(pts_x, pts_y);

  // Trigonometry to figure out a decently smooth trajectory
  // in this case "x" represents some marker 30 meters in front of the car
  double target_x = 30.0;
  double target_y = spline_fn(target_x);
  double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

  double x_add_on = 0;

  // Now generate some number of points along this path, with some spacing determined
  // between each point, based on velocity
  for (int i = 1; i < MAX_PATH_POINTS - ego.previous_path_x.size(); i++) {
    double N = (target_dist / (D_T * mps_from_mph(current_v)));
    double x_point = x_add_on + target_x / N;
    double y_point = spline_fn(x_point);
    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // Convert back to global coordinates
    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
  return {next_x_vals, next_y_vals, {target_dist}};
}

// Just a convenient wrapper to avoid having to use 6 statements to update these values
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
}

// Also a convenient wrapper to avoid multiple statements.
void Planner::update_prev_position(vector<double> new_previous_path_x,
                                   vector<double> new_previous_path_y,
                                   double new_end_path_s,
                                   double new_end_path_d) {
  ego.previous_path_x = new_previous_path_x;
  ego.previous_path_y = new_previous_path_y;
  ego.end_path_s = new_end_path_s;
  ego.end_path_d = new_end_path_d;
}

// If the current speed is too different from the target speed, then accelerate/decelerate to match.
// Basically used independent of cost functions, ensuring that the car always accelerates or slows down
// depending on the target velocity. The target velocity is determined by the plan, e.g. when approaching
// a car ahead of the ego car, then the target velocity is updated to match the car in front.
double Planner::get_updated_speed(double current_v, double target_v) {
//  printf("target: %f\n", target_v);
  double delta = (target_v - current_v);
  if (abs(delta) > 0) {
    return current_v + mph_from_mps(MAX_A * (delta > 0 ? 1 : -1)) * D_T;
  } else {
    return current_v;
  }
}

// Generates predictions for non-ego vehicles to be used in trajectory generation for the ego vehicle.
// Assumes constant velocity for other cars, and no lane changes
// sensor_fusion is a map of id to {x, y, v_x, v_y, s, d}
void Planner::predict_other_cars(map<int, vector<double>> sensor_fusion, map<int, Plan> &predictions, int num_steps) {
  for (map<int, vector<double>>::iterator it = sensor_fusion.begin();
       it != sensor_fusion.end(); ++it) {
    vector<double> measurement = it->second;
    float s = measurement[4];
    float d = measurement[5];
    float v_x = measurement[2];
    float v_y = measurement[3];

    float v = sqrt(pow(v_x, 2) + pow(v_y, 2));
    float next_s = position_at(s, v, 0, num_steps * D_T);

    predictions.insert({it->first, Plan(next_s, d, mph_from_mps(v), "KL")});
  }
}

// For lane keep we basically set the target velocity to the lowest value between the velocity we wish to go
// and the actual velocity of the lane.
void Planner::prediction_for_lane_keep(Plan &prediction,
                                       string state,
                                       double s,
                                       map<int, Plan> &predictions,
                                       int lane,
                                       double current_v) {
  double target_v = TARGET_V;
  Plan vehicle_ahead;
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

// For lane changes we actually want to set the target velociity to equal the velocity
// of the car ahead of us in the lane
void Planner::prediction_for_lane_change(Plan &prediction,
                                         string state,
                                         double s,
                                         map<int, Plan> &predictions) {
  int new_lane = ego.lane;
  if (state == "LCL") {
    new_lane -= 1;
  } else if (state == "LCR") {
    new_lane += 1;
  }

  Plan vehicle_behind;
  Plan vehicle_ahead;
  double target_v = TARGET_V;
  bool has_vehicle_ahead = false;

  has_vehicle_ahead = get_vehicle_ahead(predictions, new_lane, vehicle_ahead);
  if (has_vehicle_ahead) {
    target_v = fmin(vehicle_ahead.v, target_v);
  }

  prediction.trajectory = generate_trajectory_for_lane(ego.lane, new_lane, ego.v);
  prediction.v = target_v;
  prediction.d = new_lane * 4 + 2;
  prediction.lane = new_lane;
  prediction.s = s + prediction.trajectory[2][0];
  prediction.state = state;
}

// Given a possible next state, generate the appropriate trajectory to realize the next state.
void Planner::predict_ego(string state, Plan &candidate, map<int, Plan> &predictions) {
//  printf("eval:%s, lane:%i v:%f\n", state.c_str(), ego.lane, ego.speed);
  if (state == "KL") {
    prediction_for_lane_keep(candidate, state, ego.s, predictions, ego.lane, ego.v);
  } else if (state == "LCIP") {
    // This is the "lane change in progress" state
    prediction_for_lane_keep(candidate, state, ego.s, predictions, ego.target_lane, ego.v);
  } else if (state == "LCL" || state == "LCR") {
    prediction_for_lane_change(candidate, state, ego.s, predictions);
  }
}


// Returns true if a slower vehicle is found ahead off the current vehicle
bool Planner::get_vehicle_ahead(map<int, Plan> &predictions, int lane, Plan &vehicle_ahead) {
  // Returns true if a slower vehicle is found ahead of the current vehicle
  float min_distance = MIN_DETECTED_DISTANCE;
  bool found_vehicle = false;
  Plan temp_vehicle;
  for (map<int, Plan>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
    temp_vehicle = it->second;
    if (temp_vehicle.lane == ego.lane &&
        temp_vehicle.s > ego.s &&
        (temp_vehicle.s - ego.s) < min_distance + 25
        ) {
//      printf("vehicle ahead:%f speed: %f\n", temp_vehicle.s - ego.s, temp_vehicle.v);

//      if (temp_vehicle.v < ego.v) {
      min_distance = temp_vehicle.s - ego.s;
      vehicle_ahead = temp_vehicle;
      found_vehicle = true;
//      }
    }
  }

  return found_vehicle;
}


// Returns true if a faster vehicle is found behind the current vehicle
bool Planner::get_vehicle_behind(map<int, Plan> &predictions, int lane, Plan &vehicle_behind) {
  float min_distance = MIN_DETECTED_DISTANCE;
  bool found_vehicle = false;
  Plan temp_vehicle;
  for (map<int, Plan>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
    temp_vehicle = it->second;
    if (temp_vehicle.lane == ego.lane && temp_vehicle.s < ego.s && (ego.s - temp_vehicle.s) < min_distance && temp_vehicle.v > ego.v) {
      min_distance = temp_vehicle.s;
      vehicle_behind = temp_vehicle;
      found_vehicle = true;
    }
  }

  return found_vehicle;
}
