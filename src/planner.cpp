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

vector<vector<double>> Planner::generate_trajectory(vector<double> map_waypoints_s,
                                           vector<double> map_waypoints_x,
                                           vector<double> map_waypoints_y) {
  
  return generate_trajectory_for_lane(ego.lane, map_waypoints_s, map_waypoints_x, map_waypoints_y, TARGET_VEL);
}

vector<vector<double>> Planner::generate_trajectory_for_lane(int lane,
                                                    vector<double> map_waypoints_s,
                                                    vector<double> map_waypoints_x,
                                                    vector<double> map_waypoints_y,
                                                    float target_v) {

  // get the next 3 waypoints that we'd like to hit:
  // 30 meters, 60 meters, and 90 meters away
  vector<vector<double>> target_waypoints;
  for (int i = 30; i < 90; i+= 30) {
    target_waypoints.push_back(getXY(ego.s + i, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y));
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

  for (int i = 1; i < MAX_PATH_POINTS - ego.previous_path_x.size(); i++) {
    double N = (target_dist / (D_T * mps_from_mph(target_v)));
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
  return {next_x_vals, next_y_vals};
}

void Planner::update_position(double car_x,
                              double car_y,
                              double car_s,
                              double car_d,
                              double car_yaw,
                              double car_speed) {
  ego.x = car_x;
  ego.y = car_y;
  ego.s = car_s;
  ego.lane = (car_d / 4) + 2;
  ego.yaw = car_yaw;
  ego.speed = car_speed;
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
