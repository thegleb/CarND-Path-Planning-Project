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

// in mph
#define TARGET_VEL 49.5f
#define MAX_PATH_POINTS 50
#define D_T 0.02

using std::vector;
using std::map;

class Planner {
public:
  // Constructors
  Planner();
  virtual ~Planner();
  
  vector<vector<double>> generate_trajectory(vector<double> map_waypoints_s,
                                             vector<double> map_waypoints_x,
                                             vector<double> map_waypoints_y);
  vector<vector<double>> generate_trajectory_for_lane(int lane,
                                                      vector<double> map_waypoints_s,
                                                      vector<double> map_waypoints_x,
                                                      vector<double> map_waypoints_y,
                                                      float velocity);
  map<int, vector<double>> sensor_fusion;
};

#endif /* planner_hpp */
