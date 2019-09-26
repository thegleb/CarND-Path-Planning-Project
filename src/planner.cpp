//
//  planner.cpp
//  CarND-Path-Planning-Project
//
//  Created by Gleb on 9/24/19.
//  Copyright © 2019 Gleb. All rights reserved.
//

#include "planner.hpp"


vector<vector<double>> Planner::generate_trajectory(vector<double> map_waypoints_s,
                                           vector<double> map_waypoints_x,
                                           vector<double> map_waypoints_y) {
  
  return generate_trajectory_for_lane(1, map_waypoints_s, map_waypoints_x, map_waypoints_y, TARGET_VEL);
}

vector<vector<double>> Planner::generate_trajectory_for_lane(int lane,
                                                    vector<double> map_waypoints_s,
                                                    vector<double> map_waypoints_x,
                                                    vector<double> map_waypoints_y,
                                                    float velocity) {
  
}
